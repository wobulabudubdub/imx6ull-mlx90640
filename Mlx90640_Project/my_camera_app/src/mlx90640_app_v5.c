/*
 * mlx90640_app_v6.c - V2驱动适配版 (原始RAM数据 + 自动对比度)
 */

#define _GNU_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

#define MLX90640_DEV       "/dev/mlx90640"
#define FB_DEV             "/dev/fb0"
#define SENSOR_ROWS        24
#define SENSOR_COLS        32
#define SENSOR_PIXELS      (SENSOR_ROWS * SENSOR_COLS)
#define FRAME_WORDS        832
#define READ_FRAME_SIZE    (FRAME_WORDS * 2)
#define RAM_OFFSET_TA      0x0700
#define RAM_WORD_TA        (RAM_OFFSET_TA - 0x0400)
#define RAM_WORD_TR        (RAM_OFFSET_TA - 0x0400 + 1)

#define INVALID_MARKER     -32768
#define TEMP_MIN_VALID     -32767
#define TEMP_MAX_VALID     32766

/* 全局状态 */
static volatile int g_running = 1;
static int mlx_fd = -1;
static int fb_fd = -1;
static uint16_t *fb_base = NULL;
static int fb_width = 0;
static int fb_height = 0;
static int fb_size = 0;

static int16_t temp_frame[SENSOR_PIXELS];
static int16_t temp_sorted[SENSOR_PIXELS];
static int16_t ambient_temp = 0;
static int16_t temp_min = 0;
static int16_t temp_max = 0;
static int first_frame = 1;

/* 用于解决整数除法卡死问题的浮点跟踪器 */
static float f_temp_min = 0.0f;
static float f_temp_max = 0.0f;

static uint32_t frame_count = 0;
static struct timespec fps_clock;
static float current_fps = 0;

static void signal_handler(int sig) {
    g_running = 0;
}

static int compare_temps(const void *a, const void *b) {
    int16_t va = *(const int16_t *)a;
    int16_t vb = *(const int16_t *)b;
    return (va > vb) - (va < vb);
}

/* 经典热力渐变：蓝(冷) -> 绿(适中) -> 红(热) */
static uint16_t temp_to_color(int16_t temp) {
    float norm = 0.0f;
    
    if (temp_max > temp_min) {
        norm = (float)(temp - temp_min) / (float)(temp_max - temp_min);
    }
    
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    
    uint8_t r = 0, g = 0, b = 0;
    
    /* 划分为均匀的三个色带，平滑过渡，无黑色死角 */
    if (norm < 0.33f) {
        // 0.0 到 0.33: 纯蓝 -> 纯绿
        float t = norm / 0.33f;
        r = 0;
        g = (uint8_t)(255 * t);
        b = (uint8_t)(255 * (1.0f - t));
    } else if (norm < 0.66f) {
        // 0.33 到 0.66: 纯绿 -> 黄色(红+绿)
        float t = (norm - 0.33f) / 0.33f;
        r = (uint8_t)(255 * t);
        g = 255;
        b = 0;
    } else {
        // 0.66 到 1.0: 黄色 -> 纯红
        float t = (norm - 0.66f) / 0.34f;
        r = 255;
        g = (uint8_t)(255 * (1.0f - t));
        b = 0;
    }
    
    // 转换为 RGB565
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

static int device_init(void) {
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    
    mlx_fd = open(MLX90640_DEV, O_RDONLY);
    if (mlx_fd < 0) {
        printf("[ERROR] 无法打开 %s\n", MLX90640_DEV);
        return -1;
    }
    
    fb_fd = open(FB_DEV, O_RDWR);
    if (fb_fd < 0) {
        printf("[ERROR] 无法打开 %s\n", FB_DEV);
        close(mlx_fd);
        return -1;
    }
    
    if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0 ||
        ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo) < 0) {
        perror("ioctl");
        close(fb_fd);
        close(mlx_fd);
        return -1;
    }
    
    fb_width = vinfo.xres;
    fb_height = vinfo.yres;
    fb_size = finfo.line_length * fb_height;
    
    printf("[OK] LCD: %dx%d %dbpp\n", fb_width, fb_height, vinfo.bits_per_pixel);
    
    fb_base = (uint16_t *)mmap(NULL, fb_size, PROT_READ | PROT_WRITE,
                               MAP_SHARED, fb_fd, 0);
    if (fb_base == MAP_FAILED) {
        perror("mmap");
        close(fb_fd);
        close(mlx_fd);
        return -1;
    }
    
    memset(fb_base, 0, fb_size);
    return 0;
}

static void device_cleanup(void) {
    if (fb_base && fb_base != MAP_FAILED) {
        memset(fb_base, 0, fb_size);
        munmap(fb_base, fb_size);
    }
    if (fb_fd >= 0) close(fb_fd);
    if (mlx_fd >= 0) close(mlx_fd);
}

static int is_valid_pixel(int16_t val) {
    if (val == INVALID_MARKER) return 0;
    if (val < TEMP_MIN_VALID || val > TEMP_MAX_VALID) return 0;
    return 1;
}

static int read_frame(void) {
    uint8_t buf[READ_FRAME_SIZE];
    
    lseek(mlx_fd, 0, SEEK_SET);
    int ret = read(mlx_fd, buf, READ_FRAME_SIZE);
    if (ret != READ_FRAME_SIZE) return -1;
    
    /* V2驱动: 1664字节 = 832个uint16(raw RAM), 小端序 */
    for (int i = 0; i < SENSOR_PIXELS; i++) {
        int off = i * 2;
        uint16_t raw = (uint16_t)(buf[off] | (buf[off+1] << 8));
        /* 符号扩展: MLX90640像素数据是16位有符号 */
        int16_t val = (int16_t)raw;
        /* 转换到 0.01°C 近似值以便兼容原有逻辑 */
        temp_frame[i] = val;
    }
    
    /* 读Ta原始值作为环境参考 */
    int ta_off = RAM_WORD_TA * 2;
    uint16_t raw_ta = (uint16_t)(buf[ta_off] | (buf[ta_off+1] << 8));
    ambient_temp = (int16_t)raw_ta;
    
    return 0;
}

static void update_temp_range(void) {
    int valid_count = 0;
    
    for (int i = 0; i < SENSOR_PIXELS; i++) {
        if (is_valid_pixel(temp_frame[i])) {
            temp_sorted[valid_count++] = temp_frame[i];
        }
    }
    
    if (valid_count < 10) return;
    
    qsort(temp_sorted, valid_count, sizeof(int16_t), compare_temps);
    
    /* 排除极端噪点：取中间 96% 的数据 (跳过前2%和后2%) */
    int start_idx = valid_count * 2 / 100;
    int end_idx = valid_count * 98 / 100;
    if (end_idx >= valid_count) end_idx = valid_count - 1;
    
    int16_t min_t = temp_sorted[start_idx];
    int16_t max_t = temp_sorted[end_idx];
    
    if (first_frame) {
        f_temp_min = min_t;
        f_temp_max = max_t;
        first_frame = 0;
    } else {
        /* 重要修复：使用 float 进行低通滤波，彻底解决 int16 除法截断导致的“卡死死区” */
        f_temp_min = f_temp_min * 0.7f + (float)min_t * 0.3f;
        f_temp_max = f_temp_max * 0.7f + (float)max_t * 0.3f;
    }
    
    temp_min = (int16_t)f_temp_min;
    temp_max = (int16_t)f_temp_max;
    
    /* 强制设置最小对比度（约2摄氏度），防止对着白墙时画面全是纯色噪点 */
    if (temp_max - temp_min < 200) {
        int mid = (temp_min + temp_max) / 2;
        temp_min = mid - 100;
        temp_max = mid + 100;
    }
}

static void draw_frame(void) {
    float scale_x = (float)SENSOR_COLS / fb_width;
    float scale_y = (float)SENSOR_ROWS / fb_height;
    
    uint16_t *ptr = fb_base;
    
    for (int y = 0; y < fb_height; y++) {
        for (int x = 0; x < fb_width; x++) {
            int sx = (int)(x * scale_x);
            int sy = (int)(y * scale_y);
            
            if (sx >= SENSOR_COLS) sx = SENSOR_COLS - 1;
            if (sy >= SENSOR_ROWS) sy = SENSOR_ROWS - 1;
            
            int idx = sy * SENSOR_COLS + sx;
            int16_t val = temp_frame[idx];
            
            uint16_t color = is_valid_pixel(val) ? temp_to_color(val) : 0x0000;
            *ptr++ = color;
        }
    }
}

static void update_fps(void) {
    static int count = 0;
    struct timespec now;
    
    count++;
    if (count == 1) {
        clock_gettime(CLOCK_MONOTONIC, &fps_clock);
        return;
    }
    
    clock_gettime(CLOCK_MONOTONIC, &now);
    float elapsed = (now.tv_sec - fps_clock.tv_sec) +
                    (now.tv_nsec - fps_clock.tv_nsec) / 1e9f;
    
    if (elapsed > 1.0f) {
        current_fps = count / elapsed;
        count = 0;
        printf("[FPS] %.1f | Range: %d ~ %d | Ta(raw): 0x%04X\n",
               current_fps, temp_min, temp_max,
               (uint16_t)ambient_temp);
    }
}

int main(void) {
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  MLX90640 热成像 v6.0 (V2驱动 + 自动对比度)                ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");
    
    if (device_init() < 0) {
        return 1;
    }
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("[INFO] 开始显示... (Ctrl+C 退出)\n\n");
    
    while (g_running) {
        if (read_frame() < 0) {
            usleep(10000); // 读不到数据时，稍微等一下，别占用太多CPU
            continue;
        }
        
        update_temp_range();
        draw_frame();
        update_fps();
        
        frame_count++;
        
        // 适当休眠匹配 MLX90640 的刷新率 (通常是 8Hz 或 16Hz，约休眠30~60ms)
        usleep(30000); 
    }
    
    printf("\n[EXIT] 清理中...\n");
    device_cleanup();
    printf("[EXIT] 总共 %u 帧, FPS=%.1f\n", frame_count, current_fps);
    
    return 0;
}