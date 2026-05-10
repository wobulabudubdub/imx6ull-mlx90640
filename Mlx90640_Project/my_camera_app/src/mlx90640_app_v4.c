/*
 * mlx90640_app_v4.c - 最终版本，正确处理坏像素和特殊值
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
#define READ_FRAME_SIZE    (2 + SENSOR_PIXELS * 2)

/* 坏像素/特殊值标记 */
#define INVALID_MARKER     -4000      /* 0xF060 */
#define TEMP_MIN_VALID     -4000      /* -40°C */
#define TEMP_MAX_VALID     30500      /* 305°C */

static volatile int g_running = 1;
static int mlx_fd = -1;
static int fb_fd = -1;
static uint16_t *fb_base = NULL;
static int fb_width = 0;
static int fb_height = 0;
static int fb_size = 0;

static int16_t temp_frame[SENSOR_PIXELS];
static int16_t ambient_temp = 0;
static int16_t temp_min = 0;
static int16_t temp_max = 0;
static int first_frame = 1;

static uint32_t frame_count = 0;
static struct timespec fps_clock;
static float current_fps = 0;

static void signal_handler(int sig) {
    g_running = 0;
}

/* 温度 -> RGB565 */
static uint16_t temp_to_color(int16_t temp) {
    float norm = 0;
    
    if (temp_max > temp_min) {
        norm = (float)(temp - temp_min) / (float)(temp_max - temp_min);
    }
    
    if (norm < 0) norm = 0;
    if (norm > 1) norm = 1;
    
    uint8_t r = 0, g = 0, b = 0;
    
    if (norm < 0.25f) {
        float t = norm / 0.25f;
        b = 255;
        g = (uint8_t)(255 * t);
        r = 0;
    } else if (norm < 0.50f) {
        float t = (norm - 0.25f) / 0.25f;
        b = (uint8_t)(255 * (1 - t));
        g = 255;
        r = 0;
    } else if (norm < 0.75f) {
        float t = (norm - 0.50f) / 0.25f;
        b = 0;
        g = 255;
        r = (uint8_t)(255 * t);
    } else {
        float t = (norm - 0.75f) / 0.25f;
        b = 0;
        g = (uint8_t)(255 * (1 - t));
        r = 255;
    }
    
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

static int device_init(void) {
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    
    mlx_fd = open(MLX90640_DEV, O_RDONLY);
    if (mlx_fd < 0) {
        printf("[ERROR] 打开 %s 失败\n", MLX90640_DEV);
        return -1;
    }
    printf("[OK] MLX90640 已打开\n");
    
    fb_fd = open(FB_DEV, O_RDWR);
    if (fb_fd < 0) {
        printf("[ERROR] 打开 %s 失败\n", FB_DEV);
        close(mlx_fd);
        return -1;
    }
    
    if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0 ||
        ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo) < 0) {
        perror("ioctl 失败");
        close(fb_fd);
        close(mlx_fd);
        return -1;
    }
    
    fb_width = vinfo.xres;
    fb_height = vinfo.yres;
    fb_size = finfo.line_length * fb_height;
    
    printf("[INFO] LCD: %dx%d %dbpp\n", fb_width, fb_height, vinfo.bits_per_pixel);
    
    fb_base = (uint16_t *)mmap(NULL, fb_size, PROT_READ | PROT_WRITE,
                               MAP_SHARED, fb_fd, 0);
    if (fb_base == MAP_FAILED) {
        perror("mmap 失败");
        close(fb_fd);
        close(mlx_fd);
        return -1;
    }
    
    memset(fb_base, 0, fb_size);
    printf("[OK] Framebuffer 已映射\n");
    
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

/* 检查像素是否有效 */
static int is_valid_pixel(int16_t val) {
    /* 过滤掉标记值 */
    if (val == INVALID_MARKER) return 0;
    
    /* 过滤掉超出范围的值 */
    if (val < TEMP_MIN_VALID || val > TEMP_MAX_VALID) return 0;
    
    return 1;
}

static int read_frame(void) {
    uint8_t buf[READ_FRAME_SIZE];
    int ret = read(mlx_fd, buf, READ_FRAME_SIZE);
    
    if (ret != READ_FRAME_SIZE) {
        return -1;
    }
    
    ambient_temp = (int16_t)(buf[0] | (buf[1] << 8));
    
    for (int i = 0; i < SENSOR_PIXELS; i++) {
        int offset = 2 + i * 2;
        temp_frame[i] = (int16_t)(buf[offset] | (buf[offset + 1] << 8));
    }
    
    return 0;
}

static void update_temp_range(void) {
    int16_t min_t = INT16_MAX;
    int16_t max_t = INT16_MIN;
    int valid_count = 0;
    
    /* 只对有效像素计算范围 */
    for (int i = 0; i < SENSOR_PIXELS; i++) {
        if (!is_valid_pixel(temp_frame[i])) continue;
        
        if (temp_frame[i] < min_t) min_t = temp_frame[i];
        if (temp_frame[i] > max_t) max_t = temp_frame[i];
        valid_count++;
    }
    
    if (valid_count == 0) return;  /* 没有有效像素 */
    
    if (first_frame) {
        temp_min = min_t;
        temp_max = max_t;
        first_frame = 0;
        printf("[INIT] 有效像素: %d/%d\n", valid_count, SENSOR_PIXELS);
        printf("[INIT] 温度范围: %.2f ~ %.2f°C\n",
               temp_min/100.0f, temp_max/100.0f);
    } else {
        /* 快速跟踪 */
        if (min_t < temp_min) {
            temp_min = min_t;
        } else {
            temp_min = (temp_min * 9 + min_t) / 10;
        }
        
        if (max_t > temp_max) {
            temp_max = max_t;
        } else {
            temp_max = (temp_max * 9 + max_t) / 10;
        }
    }
    
    /* 防止范围过小 */
    if (temp_max - temp_min < 100) {
        int mid = (temp_min + temp_max) / 2;
        temp_min = mid - 50;
        temp_max = mid + 50;
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
            
            /* 无效像素显示为黑色 */
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
        printf("[FPS] %.1f | Temp: %.2f ~ %.2f°C | Amb: %.2f°C\n",
               current_fps, temp_min/100.0f, temp_max/100.0f,
               ambient_temp/100.0f);
    }
}

int main(void) {
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  MLX90640 热成像显示 v4.0 (坏像素过滤版)                ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");
    
    if (device_init() < 0) {
        return 1;
    }
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("[INFO] 开始显示... (Ctrl+C 退出)\n\n");
    
    while (g_running) {
        if (read_frame() < 0) {
            usleep(50000);
            continue;
        }
        
        update_temp_range();
        draw_frame();
        update_fps();
        
        frame_count++;
        
        if (frame_count % 50 == 0) {
            printf("[FRAME %u] 环境温度: %.2f°C\n", 
                   frame_count, ambient_temp/100.0f);
        }
        
        usleep(30000);
    }
    
    printf("\n[EXIT] 清理中...\n");
    device_cleanup();
    printf("[EXIT] 总共 %u 帧, FPS=%.1f\n", frame_count, current_fps);
    
    return 0;
}
