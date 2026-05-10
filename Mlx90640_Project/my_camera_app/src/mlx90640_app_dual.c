/*
 * mlx90640_app_dual.c - OV5640 + MLX90640 双画面显示
 *
 * 屏幕1024x600:
 *   左半512x600 = OV5640摄像头图像
 *   右半512x600 = MLX90640热成像
 *
 * 双线程架构: 各管半边fb, 无锁无竞争
 */

#define _GNU_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

/* ========== 硬件设备路径 ========== */
#define MLX90640_DEV       "/dev/mlx90640"
#define FB_DEV             "/dev/fb0"
#define CAM_DEV0           "/dev/video1"
#define CAM_DEV1           "/dev/video0"

/* ========== 显示参数 ========== */
#define FB_WIDTH           1024
#define FB_HEIGHT          600
#define HALF_WIDTH         (FB_WIDTH / 2)

/* ========== MLX90640 参数 ========== */
#define SENSOR_ROWS        24
#define SENSOR_COLS        32
#define SENSOR_PIXELS      (SENSOR_ROWS * SENSOR_COLS)
#define FRAME_WORDS        832
#define MLX_FRAME_SIZE     (FRAME_WORDS * 2)
#define RAM_WORD_TA        768

/* ========== V4L2 参数 ========== */
#define CAM_WIDTH          640
#define CAM_HEIGHT         480
#define V4L2_BUF_COUNT     4

/* ========== 全局状态 ========== */
static volatile int g_running = 1;
static uint16_t *fb_base;
static int fb_line_len;

/* MLX90640 数据 */
static int16_t temp_frame[SENSOR_PIXELS];
static int16_t temp_sorted[SENSOR_PIXELS];
static int16_t ambient_temp;
static int16_t temp_min, temp_max;
static int first_frame = 1;

static void signal_handler(int sig) { g_running = 0; }

/* ========== 工具函数 ========== */

static int compare_temps(const void *a, const void *b) {
    int16_t va = *(const int16_t *)a;
    int16_t vb = *(const int16_t *)b;
    return (va > vb) - (va < vb);
}

/* YUYV 4:2:2 → RGB565, 查表法加速 */
static inline uint16_t yuv_to_rgb565(uint8_t y, uint8_t u, uint8_t v) {
    int c = y - 16;
    int d = u - 128;
    int e = v - 128;
    int r = ((298 * c + 409 * e + 128) >> 8);
    int g = ((298 * c - 100 * d - 208 * e + 128) >> 8);
    int b = ((298 * c + 516 * d + 128) >> 8);
    if (r < 0) r = 0; if (r > 255) r = 255;
    if (g < 0) g = 0; if (g > 255) g = 255;
    if (b < 0) b = 0; if (b > 255) b = 255;
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

/* ========== 帧缓冲 ========== */

static int fb_init(void) {
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    int fd, size;

    fd = open(FB_DEV, O_RDWR);
    if (fd < 0) { perror("open fb0"); return -1; }

    if (ioctl(fd, FBIOGET_VSCREENINFO, &vinfo) < 0 ||
        ioctl(fd, FBIOGET_FSCREENINFO, &finfo) < 0) {
        perror("fb ioctl"); close(fd); return -1;
    }

    fb_line_len = finfo.line_length;
    size = fb_line_len * vinfo.yres;

    printf("[FB] %dx%d line=%d bpp=%d\n",
           vinfo.xres, vinfo.yres, fb_line_len, vinfo.bits_per_pixel);

    fb_base = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (fb_base == MAP_FAILED) {
        perror("mmap fb"); close(fd); return -1;
    }
    memset(fb_base, 0, size);
    return fd;
}

/* ========== MLX90640 逻辑 ========== */

static uint16_t temp_to_color(int16_t t) {
    float norm = 0;
    if (temp_max > temp_min)
        norm = (float)(t - temp_min) / (float)(temp_max - temp_min);
    if (norm < 0) norm = 0;
    if (norm > 1) norm = 1;

    uint8_t r = 0, g = 0, b = 0;
    if (norm < 0.33f) {
        float s = norm / 0.33f;
        r = 0; g = (uint8_t)(255 * s); b = (uint8_t)(255 * (1 - s));
    } else if (norm < 0.66f) {
        float s = (norm - 0.33f) / 0.33f;
        r = (uint8_t)(255 * s); g = 255; b = 0;
    } else {
        float s = (norm - 0.66f) / 0.34f;
        r = 255; g = (uint8_t)(255 * (1 - s)); b = 0;
    }
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

static int mlx_read_frame(int fd) {
    uint8_t buf[MLX_FRAME_SIZE];
    lseek(fd, 0, SEEK_SET);
    if (read(fd, buf, MLX_FRAME_SIZE) != MLX_FRAME_SIZE) return -1;

    for (int i = 0; i < SENSOR_PIXELS; i++) {
        int off = i * 2;
        uint16_t raw = (uint16_t)(buf[off] | (buf[off+1] << 8));
        temp_frame[i] = (int16_t)raw;
    }
    int ta_off = RAM_WORD_TA * 2;
    ambient_temp = (int16_t)(uint16_t)(buf[ta_off] | (buf[ta_off+1] << 8));
    return 0;
}

static void mlx_update_range(void) {
    int valid_count = 0;
    for (int i = 0; i < SENSOR_PIXELS; i++)
        if (temp_frame[i] != -32768)
            temp_sorted[valid_count++] = temp_frame[i];

    if (valid_count < 10) return;
    qsort(temp_sorted, valid_count, sizeof(int16_t), compare_temps);

    int start = valid_count * 2 / 100;
    int end = valid_count * 98 / 100;
    if (end >= valid_count) end = valid_count - 1;

    int16_t min_t = temp_sorted[start];
    int16_t max_t = temp_sorted[end];

    if (first_frame) {
        temp_min = min_t; temp_max = max_t; first_frame = 0;
    } else {
        temp_min = (temp_min + min_t) / 2;
        temp_max = (temp_max + max_t) / 2;
    }
    if (temp_max - temp_min < 200) {
        int mid = (temp_min + temp_max) / 2;
        temp_min = mid - 100; temp_max = mid + 100;
    }
}

static void mlx_draw_right(void) {
    int bw = HALF_WIDTH / SENSOR_COLS;
    int bh = FB_HEIGHT / SENSOR_ROWS;

    for (int py = 0; py < FB_HEIGHT; py++) {
        int sy = py / bh;
        if (sy >= SENSOR_ROWS) sy = SENSOR_ROWS - 1;

        uint16_t *line = (uint16_t *)((uint8_t *)fb_base + py * fb_line_len);
        line += HALF_WIDTH;

        for (int px = 0; px < HALF_WIDTH; px++) {
            int sx = px / bw;
            if (sx >= SENSOR_COLS) sx = SENSOR_COLS - 1;
            line[px] = temp_to_color(temp_frame[sy * SENSOR_COLS + sx]);
        }
    }
}

static void *mlx_thread(void *arg) {
    int fd = *(int *)arg;
    printf("[MLX] Thread started\n");

    while (g_running) {
        if (mlx_read_frame(fd) == 0) {
            mlx_update_range();
            mlx_draw_right();
        }
        usleep(50000);
    }
    return NULL;
}

/* ========== OV5640 V4L2 ========== */

static void *cam_bufs[V4L2_BUF_COUNT];
static int cam_buf_lens[V4L2_BUF_COUNT];
static int cam_w = CAM_WIDTH, cam_h = CAM_HEIGHT;
static uint32_t cam_fmt = V4L2_PIX_FMT_YUYV;

static int cam_open(void) {
    int fd = open(CAM_DEV0, O_RDWR | O_NONBLOCK);
    if (fd < 0) fd = open(CAM_DEV1, O_RDWR | O_NONBLOCK);
    return fd;
}

static int cam_init(int fd) {
    struct v4l2_capability cap;
    struct v4l2_format fmt;

    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("QUERYCAP"); return -1;
    }
    printf("[CAM] Driver: %s  Card: %s\n", cap.driver, cap.card);

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = CAM_WIDTH;
    fmt.fmt.pix.height = CAM_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            perror("S_FMT"); return -1;
        }
    }

    cam_fmt = fmt.fmt.pix.pixelformat;
    cam_w = fmt.fmt.pix.width;
    cam_h = fmt.fmt.pix.height;
    printf("[CAM] Format: %s %dx%d\n",
           cam_fmt == V4L2_PIX_FMT_YUYV ? "YUYV" : "RGB565", cam_w, cam_h);

    struct v4l2_requestbuffers req = {0};
    req.count = V4L2_BUF_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("REQBUFS"); return -1;
    }

    for (int i = 0; i < (int)req.count; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("QUERYBUF"); return -1;
        }
        cam_bufs[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd, buf.m.offset);
        cam_buf_lens[i] = buf.length;
        if (cam_bufs[i] == MAP_FAILED) { perror("mmap cam"); return -1; }
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("QBUF"); return -1;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("STREAMON"); return -1;
    }
    printf("[CAM] Streaming started\n");
    return 0;
}

/* YUYV帧 → RGB565 写入framebuffer左半 (最近邻缩放) */
static void cam_draw_yuyv(const uint8_t *yuv, int src_w, int src_h) {
    int dst_w = HALF_WIDTH, dst_h = FB_HEIGHT;
    float sx = (float)src_w / dst_w;
    float sy = (float)src_h / dst_h;

    for (int y = 0; y < dst_h; y++) {
        int syi = (int)(y * sy);
        if (syi >= src_h) syi = src_h - 1;

        const uint8_t *row = yuv + syi * src_w * 2;
        uint16_t *line = (uint16_t *)((uint8_t *)fb_base + y * fb_line_len);

        for (int x = 0; x < dst_w; x++) {
            int sxi = (int)(x * sx);
            if (sxi >= src_w) sxi = src_w - 1;

            int idx = (sxi / 2) * 4;
            uint8_t y0 = row[idx];
            uint8_t u  = row[idx + 1];
            uint8_t y1 = row[idx + 2];
            uint8_t v  = row[idx + 3];

            line[x] = (sxi & 1) ? yuv_to_rgb565(y1, u, v)
                                : yuv_to_rgb565(y0, u, v);
        }
    }
}

/* RGB565帧 → 直接写入framebuffer左半 (最近邻缩放) */
static void cam_draw_rgb565(const uint8_t *rgb, int src_w, int src_h) {
    int dst_w = HALF_WIDTH, dst_h = FB_HEIGHT;
    float sx = (float)src_w / dst_w;
    float sy = (float)src_h / dst_h;

    for (int y = 0; y < dst_h; y++) {
        int syi = (int)(y * sy);
        if (syi >= src_h) syi = src_h - 1;
        uint16_t *src_line = (uint16_t *)(rgb + syi * src_w * 2);
        uint16_t *dst_line = (uint16_t *)((uint8_t *)fb_base + y * fb_line_len);

        for (int x = 0; x < dst_w; x++) {
            int sxi = (int)(x * sx);
            if (sxi >= src_w) sxi = src_w - 1;
            dst_line[x] = src_line[sxi];
        }
    }
}

static void *cam_thread(void *arg) {
    int fd = *(int *)arg;
    printf("[CAM] Thread started: %dx%d fmt=0x%X\n", cam_w, cam_h, cam_fmt);

    while (g_running) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            usleep(10000);
            continue;
        }

        uint8_t *data = (uint8_t *)cam_bufs[buf.index];

        if (cam_fmt == V4L2_PIX_FMT_YUYV)
            cam_draw_yuyv(data, cam_w, cam_h);
        else
            cam_draw_rgb565(data, cam_w, cam_h);

        ioctl(fd, VIDIOC_QBUF, &buf);
    }
    return NULL;
}

/* ========== 主函数 ========== */

int main(void) {
    printf("══╦══════════════════════════════════════════╗\n");
    printf("  ║  OV5640 + MLX90640 双画面显示 v1.0     ║\n");
    printf("══╩══════════════════════════════════════════╝\n\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int fb_fd = fb_init();
    if (fb_fd < 0) return 1;

    int mlx_fd = open(MLX90640_DEV, O_RDONLY);
    if (mlx_fd < 0) {
        perror("open mlx90640");
        printf("[WARN] MLX90640 not available, thermal side disabled\n");
    } else {
        printf("[MLX] /dev/mlx90640 opened\n");
    }

    int cam_fd = cam_open();
    if (cam_fd < 0) {
        perror("open video");
        printf("[WARN] No camera device found\n");
    } else {
        printf("[CAM] Camera device opened\n");
        if (cam_init(cam_fd) < 0) {
            printf("[WARN] Camera init failed, closing\n");
            close(cam_fd); cam_fd = -1;
        }
    }

    pthread_t tid_mlx, tid_cam;

    if (mlx_fd >= 0)
        pthread_create(&tid_mlx, NULL, mlx_thread, &mlx_fd);
    if (cam_fd >= 0)
        pthread_create(&tid_cam, NULL, cam_thread, &cam_fd);

    printf("[INFO] Running... Ctrl+C to exit\n\n");

    while (g_running) usleep(100000);

    if (mlx_fd >= 0) pthread_join(tid_mlx, NULL);
    if (cam_fd >= 0) pthread_join(tid_cam, NULL);

    if (mlx_fd >= 0) close(mlx_fd);
    if (cam_fd >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam_fd, VIDIOC_STREAMOFF, &type);
        close(cam_fd);
    }
    close(fb_fd);
    printf("[EXIT] Done\n");
    return 0;
}
