#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define DEV             "/dev/mlx90640"
#define FRAME_WORDS     832
#define FRAME_BYTES     (FRAME_WORDS * 2)
#define PIXEL_WORDS     768

#define RAM_OFFSET_TA   0x0700
#define RAM_WORD_TA     (RAM_OFFSET_TA - 0x0400)

static uint16_t read_u16le(const uint8_t *buf, int word_idx)
{
    int off = word_idx * 2;
    return (uint16_t)(buf[off] | (buf[off+1] << 8));
}

static void dump_hex(const uint8_t *buf, int len, const char *label)
{
    int i;
    printf("\n--- %s (%d bytes) ---\n", label, len);
    for (i = 0; i < len; i++) {
        printf("%02X ", buf[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    if (len % 16) printf("\n");
}

static int compare_bufs(const uint8_t *a, const uint8_t *b, int len)
{
    int i, diffs = 0;
    for (i = 0; i < len; i++)
        if (a[i] != b[i]) diffs++;
    return diffs;
}

static int compare_pixel_words(const uint8_t *a, const uint8_t *b)
{
    int i, diffs = 0;
    for (i = 0; i < PIXEL_WORDS; i++)
        if (read_u16le(a, i) != read_u16le(b, i)) diffs++;
    return diffs;
}

static void print_frame_stats(const uint8_t *frame, int frame_nr)
{
    uint16_t ta = read_u16le(frame, RAM_WORD_TA);
    uint16_t subpage = read_u16le(frame, RAM_WORD_TA + 1) & 0x0001;

    printf("[Frame#%d] Ta=0x%04X (%d)  Subpage=%d\n",
           frame_nr, ta, ta, subpage);

    uint32_t sum = 0;
    uint16_t vmin = 0xFFFF, vmax = 0;

    for (int i = 0; i < PIXEL_WORDS; i++) {
        uint16_t v = read_u16le(frame, i);
        sum += v;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }
    printf("  Pixels: min=%u max=%u avg=%.0f span=%u\n",
           vmin, vmax, sum / (float)PIXEL_WORDS, vmax - vmin);
    printf("  First 8 pix: ");
    for (int i = 0; i < 8; i++)
        printf("0x%04X ", read_u16le(frame, i));
    printf("\n");
}

int main(void)
{
    int fd, ret, frame_nr;
    uint8_t prev[FRAME_BYTES], cur[FRAME_BYTES];

    fd = open(DEV, O_RDONLY);
    if (fd < 0) {
        perror("open");
        printf("\n[HINT] Is driver loaded?\n");
        return 1;
    }

    printf("=== MLX90640 Raw RAM Dump ===\n");
    printf("Device: %s\n", DEV);
    printf("Frame:  %d words (%d bytes)\n", FRAME_WORDS, FRAME_BYTES);

    ret = read(fd, cur, FRAME_BYTES);
    if (ret != FRAME_BYTES) {
        printf("[ERROR] Read returned %d (expected %d): %s\n",
               ret, FRAME_BYTES, strerror(errno));
        close(fd);
        return 1;
    }

    print_frame_stats(cur, 1);
    dump_hex(cur, 64, "Frame#1 first 64 bytes");
    memcpy(prev, cur, FRAME_BYTES);
    int last_changed = 1;

    for (frame_nr = 2; frame_nr <= 10; frame_nr++) {
        ret = read(fd, cur, FRAME_BYTES);
        if (ret != FRAME_BYTES) {
            printf("[Frame#%d] read returned %d: %s\n",
                   frame_nr, ret, strerror(errno));
            continue;
        }

        int byte_diffs = compare_bufs(prev, cur, FRAME_BYTES);
        int pixel_diffs = compare_pixel_words(prev, cur);

        if (byte_diffs == 0) {
            printf("[Frame#%d] SAME as #%d\n", frame_nr, last_changed);
        } else {
            printf("[Frame#%d] CHANGED: %d bytes / %d pix vs #%d\n",
                   frame_nr, byte_diffs, pixel_diffs, last_changed);
            last_changed = frame_nr;
        }

        if (byte_diffs > 0 && frame_nr % 3 == 0)
            print_frame_stats(cur, frame_nr);

        memcpy(prev, cur, FRAME_BYTES);
    }

    close(fd);
    return 0;
}
