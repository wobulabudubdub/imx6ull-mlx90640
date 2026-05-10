/*
 * mlx90640_app_v3.c - MLX90640 热成像显示应用 (重新设计版)
 *
 * 问题修复:
 * - 正确的温度范围自适应
 * - 高效的缩放算法
 * - 及时的framebuffer刷新
 * - 调试信息更详细
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

/* 全局变量 */
static volatile int g_running = 1;
static int mlx_fd = -1;
static int fb_fd = -1;
static uint16_t *fb_base = NULL;
static int fb_width = 0;
static int fb_height = 0;
static int fb_size = 0;

/* 传感器数据 */
static int16_t temp_frame[SENSOR_PIXELS];
static int16_t ambient_temp = 0;
static int16_t temp_min = 0;
static int16_t temp_max = 0;
static int first_frame = 1;

/* 统计 */
static uint32_t frame_count = 0;
static struct timespec fps_clock;
static float current_fps = 0;

static void signal_handler(int sig) {
    g_running = 0;
}

/* 温度 -> RGB565 彩虹色 */
static uint16_t temp_to_color(int16_t temp) {
    float norm = 0;
    
    if (temp_max > temp_min) {
        norm = (float)(temp - temp_min) / (float)(temp_max - temp_min);
    }
    
    if (norm < 0) norm = 0;
    if (norm > 1) norm = 1;
    
    uint8_t r = 0, g = 0, b = 0;
    
    /* 5级彩虹: 蓝->青->绿->黄->红 */
    if (norm < 0.25f) {
        /* 蓝 -> 青 */
        float t = norm / 0.25f;
        b = 255;
        g = (uint8_t)(255 * t);
        r = 0;
    } else if (norm < 0.50f) {
        /* 青 -> 绿 */
        float t = (norm - 0.25f) / 0.25f;
        b = (uint8_t)(255 * (1 - t));
        g = 255;
        r = 0;
    } else if (norm < 0.75f) {
        /* 绿 -> 黄 */
        float t = (norm - 0.50f) / 0.25f;
        b = 0;
        g = 255;
        r = (uint8_t)(255 * t);
    } else {
        /* 黄 -> 红 */
        float t = (norm - 0.75f) / 0.25f;
        b = 0;
        g = (uint8_t)(255 * (1 - t));
        r = 255;
    }
    
    /* RGB565: RRRRRGGGGGGBBBBB */
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

/* 打开设备 */
static int device_init(void) {
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    
    mlx_fd = open(MLX90640_DEV, O_RDONLY);
    if (mlx_fd < 0) {
        printf("[ERROR] 无法打开 %s\n", MLX90640_DEV);
        return -1;
    }
    printf("[OK] 已打开 MLX90640 设备\n");
    
    fb_fd = open(FB_DEV, O_RDWR);
    if (fb_fd < 0) {
        printf("[ERROR] 无法打开 %s\n", FB_DEV);
        close(mlx_fd);
        return -1;
    }
    
    if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0) {
        perror("[ERROR] FBIOGET_VSCREENINFO");
        close(fb_fd);
        close(mlx_fd);
        return -1;
    }
    
    if (ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo) < 0) {
        perror("[ERROR] FBIOGET_FSCREENINFO");
        close(fb_fd);
        close(mlx_fd);
        return -1;
    }
    
    fb_width = vinfo.xres;
    fb_height = vinfo.yres;
    fb_size = finfo.line_length * fb_height;
    
    printf("[INFO] LCD: %dx%d, %d bpp\n", fb_width, fb_height, vinfo.bits_per_pixel);
    
    fb_base = (uint16_t *)mmap(NULL, fb_size, PROT_READ | PROT_WRITE, 
                               MAP_SHARED, fb_fd, 0);
    if (fb_base == MAP_FAILED) {
        perror("[ERROR] mmap failed");
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

/* 读取一帧数据 */
static int read_frame(void) {
    uint8_t buf[READ_FRAME_SIZE];
    int ret = read(mlx_fd, buf, READ_FRAME_SIZE);
    
    if (ret != READ_FRAME_SIZE) {
        printf("[WARN] 读取大小不匹配: %d/%d\n", ret, READ_FRAME_SIZE);
        return -1;
    }
    
    /* 解析环境温度 (小端序 int16) */
    ambient_temp = buf[0] | (buf[1] << 8);
    if (ambient_temp & 0x8000) ambient_temp = -(0x10000 - ambient_temp);
    
    /* 解析768个像素 */
    for (int i = 0; i < SENSOR_PIXELS; i++) {
        int offset = 2 + i * 2;
        int16_t val = buf[offset] | (buf[offset + 1] << 8);
        if (val & 0x8000) val = -(0x10000 - val);
        temp_frame[i] = val;
    }
    
    return 0;
}

/* 更新温度范围 */
static void update_temp_range(void) {
    int16_t min_t = temp_frame[0];
    int16_t max_t = temp_frame[0];
    
    for (int i = 1; i < SENSOR_PIXELS; i++) {
        if (temp_frame[i] < min_t) min_t = temp_frame[i];
        if (temp_frame[i] > max_t) max_t = temp_frame[i];
    }
    
    /* 第一帧：初始化范围 */
    if (first_frame) {
        temp_min = min_t;
        temp_max = max_t;
        first_frame = 0;
        printf("[INIT] 温度范围: %.2f ~ %.2f °C\n", 
               temp_min/100.0f, temp_max/100.0f);
    } else {
        /* 后续帧：平滑更新 */
        /* 快速跟踪最小值 */
        if (min_t < temp_min) {
            temp_min = min_t;
        } else {
            temp_min = (temp_min * 9 + min_t) / 10;
        }
        
        /* 快速跟踪最大值 */
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

/* 绘制到framebuffer - 使用最近邻采样 */
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
            uint16_t color = temp_to_color(temp_frame[idx]);
            
            *ptr++ = color;
        }
    }
}

/* 计算FPS */
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
        printf("[INFO] FPS=%.1f, Tmin=%.2f, Tmax=%.2f, Amb=%.2f\n",
               current_fps, 
               temp_min/100.0f, 
               temp_max/100.0f,
               ambient_temp/100.0f);
    }
}

int main(void) {
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  MLX90640 热成像显示 v3.0 (修复版)                        ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");
    
    if (device_init() < 0) {
        return 1;
    }
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("[INFO] 开始显示... (Ctrl+C 退出)\n\n");
    
    while (g_running) {
        /* 读取传感器数据 */
        if (read_frame() < 0) {
            usleep(50000);
            continue;
        }
        
        /* 更新温度范围 */
        update_temp_range();
        
        /* 绘制到LCD */
        draw_frame();
        
        /* 统计FPS */
        update_fps();
        
        frame_count++;
        
        /* 每50帧打印详细信息 */
        if (frame_count % 50 == 0) {
            printf("[FRAME %u] Amb=%.2f°C, Range=[%.2f, %.2f]°C, Pixel[0]=%.2f°C\n",
                   frame_count,
                   ambient_temp/100.0f,
                   temp_min/100.0f,
                   temp_max/100.0f,
                   temp_frame[0]/100.0f);
        }
        
        usleep(30000);  /* 约33fps */
    }
    
    printf("\n[EXIT] 正在清理...\n");
    device_cleanup();
    printf("[EXIT] 总共显示 %u 帧, 最终 FPS=%.1f\n", frame_count, current_fps);
    
    return 0;
}
