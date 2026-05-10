/*
 * mlx90640_app.c - MLX90640 热成像数据显示应用 (v2.0)
 *
 * 功能：
 * - 读取 /dev/mlx90640 的温度数据 (驱动输出: 环境温度2字节 + 768像素×2字节)
 * - 温度数据单位: 0.01°C (int16_t 格式)
 * - 转换为热成像图像，显示到 LCD 屏幕 (/dev/fb0)
 * - LCD 屏幕: RGB565 格式，通过 framebuffer 接口
 * 
 * MLX90640 传感器规格:
 * - 分辨率: 32x24 像素
 * - 温度范围: -40°C ~ 305°C
 * - 帧速率: 可配置 1-64Hz
 * 
 * 用法:
 *   ./camera_app              # 自动缩放温度范围
 *   ./camera_app --fixed      # 固定温度范围 -40C ~ 305C
 *   ./camera_app --verbose    # 显示详细调试信息
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <signal.h>

#define MLX90640_DEV          "/dev/mlx90640"
#define FB_DEV                "/dev/fb0"
#define MLX90640_ROWS         24
#define MLX90640_COLS         32
#define MLX90640_PIXELS       (MLX90640_ROWS * MLX90640_COLS)
#define MLX90640_FRAME_SIZE   (2 + MLX90640_PIXELS * 2)  /* 环境温度(2) + 像素(768*2) = 1538字节 */

/* 全局标志 */
static volatile int g_running = 1;
static int g_verbose = 0;
static int g_use_auto_scale = 1;

/* LCD 帧缓冲信息 */
static int fb_fd = -1;
static unsigned short *fb_base = NULL;
static int fb_width = 0;
static int fb_height = 0;
static long fb_size = 0;

/* 温度数据 */
static int16_t temp_data[MLX90640_PIXELS];
static int16_t ambient_temp = 0;  /* 环境温度 */
static int16_t temp_min = 2000;   /* 初始范围: 20.00°C */
static int16_t temp_max = 3000;   /* 初始范围: 30.00°C */

/* 性能统计 */
static uint32_t frame_count = 0;
static struct timespec fps_start;
static float current_fps = 0.0f;

/* 信号处理 */
static void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        g_running = 0;
        printf("\n[SIG] 收到退出信号，清理中...\n");
    }
}

/* 彩虹色标：将温度值 (0.01°C单位) 映射到 RGB565 颜色 */
static uint16_t temp_to_rgb565(int16_t temp) {
    /* 将温度值归一化到 [0, 1] 区间 */
    float t;
    if (temp_max > temp_min) {
        t = (float)(temp - temp_min) / (float)(temp_max - temp_min);
    } else {
        t = 0.5f;
    }
    
    /* 限制在 [0, 1] 范围内 */
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    
    /* 彩虹色映射: 蓝色 -> 青色 -> 绿色 -> 黄色 -> 红色 */
    uint8_t r, g, b;
    
    if (t < 0.2f) {
        /* 蓝色到青色 */
        float t0 = t / 0.2f;
        r = 0;
        g = (uint8_t)(255.0f * t0);
        b = 255;
    } else if (t < 0.4f) {
        /* 青色到绿色 */
        float t0 = (t - 0.2f) / 0.2f;
        r = 0;
        g = 255;
        b = (uint8_t)(255.0f * (1.0f - t0));
    } else if (t < 0.6f) {
        /* 绿色到黄色 */
        float t0 = (t - 0.4f) / 0.2f;
        r = (uint8_t)(255.0f * t0);
        g = 255;
        b = 0;
    } else if (t < 0.8f) {
        /* 黄色到橙色 */
        float t0 = (t - 0.6f) / 0.2f;
        r = 255;
        g = (uint8_t)(255.0f * (1.0f - t0 * 0.3f));
        b = 0;
    } else {
        /* 橙色到红色 */
        float t0 = (t - 0.8f) / 0.2f;
        r = 255;
        g = (uint8_t)(255.0f * (0.7f - t0 * 0.3f));
        b = (uint8_t)(255.0f * t0 * 0.2f);
    }
    
    /* 转换为 RGB565 格式 */
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

/* 初始化 framebuffer 设备 */
static int fb_init(void) {
    struct fb_var_screeninfo vscreen;
    struct fb_fix_screeninfo fscreen;
    
    fb_fd = open(FB_DEV, O_RDWR);
    if (fb_fd < 0) {
        perror("[ERROR] 打开framebuffer设备失败");
        return -1;
    }
    
    if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vscreen) < 0) {
        perror("[ERROR] 获取可变屏幕信息失败");
        close(fb_fd);
        fb_fd = -1;
        return -1;
    }
    
    if (ioctl(fb_fd, FBIOGET_FSCREENINFO, &fscreen) < 0) {
        perror("[ERROR] 获取固定屏幕信息失败");
        close(fb_fd);
        fb_fd = -1;
        return -1;
    }
    
    fb_width = vscreen.xres;
    fb_height = vscreen.yres;
    fb_size = fscreen.line_length * fb_height;
    
    printf("[INFO] 显示屏: %dx%d, %dbpp, line_length=%d\n", 
           fb_width, fb_height, vscreen.bits_per_pixel, fscreen.line_length);
    
    if (vscreen.bits_per_pixel != 16) {
        printf("[WARN] 期望16bit像素，当前是%dbpp\n", vscreen.bits_per_pixel);
    }
    
    fb_base = (unsigned short *)mmap(NULL, fb_size, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, fb_fd, 0);
    if (fb_base == MAP_FAILED) {
        perror("[ERROR] mmap framebuffer失败");
        close(fb_fd);
        fb_fd = -1;
        return -1;
    }
    
    memset(fb_base, 0, fb_size);
    return 0;
}

/* 清理 framebuffer */
static void fb_cleanup(void) {
    if (fb_base != NULL && fb_base != MAP_FAILED) {
        memset(fb_base, 0, fb_size);
        munmap(fb_base, fb_size);
        fb_base = NULL;
    }
    if (fb_fd >= 0) {
        close(fb_fd);
        fb_fd = -1;
    }
}

/* 双线性插值：从 32x24 传感器数据插值到任意分辨率 */
static inline float bilinear_interp(float x, float y, int16_t *src, int sw, int sh) {
    int x0 = (int)x;
    int y0 = (int)y;
    int x1 = (x0 + 1 < sw) ? x0 + 1 : x0;
    int y1 = (y0 + 1 < sh) ? y0 + 1 : y0;
    
    float dx = x - x0;
    float dy = y - y0;
    
    float v00 = (float)src[y0 * sw + x0];
    float v10 = (float)src[y0 * sw + x1];
    float v01 = (float)src[y1 * sw + x0];
    float v11 = (float)src[y1 * sw + x1];
    
    float v0 = v00 * (1.0f - dx) + v10 * dx;
    float v1 = v01 * (1.0f - dx) + v11 * dx;
    return v0 * (1.0f - dy) + v1 * dy;
}

/* 绘制热成像图像到 framebuffer */
static void draw_thermal_image(void) {
    int x, y;
    int sw = MLX90640_COLS;
    int sh = MLX90640_ROWS;
    
    /* 使用双线性插值进行缩放 */
    for (y = 0; y < fb_height; y++) {
        float src_y = (float)y * (sh - 1) / (fb_height - 1);
        for (x = 0; x < fb_width; x++) {
            float src_x = (float)x * (sw - 1) / (fb_width - 1);
            float temp_val = bilinear_interp(src_x, src_y, temp_data, sw, sh);
            fb_base[y * fb_width + x] = temp_to_rgb565((int16_t)temp_val);
        }
    }
}

/* 绘制温度色标条 */
static void draw_colorbar(void) {
    int bar_x = fb_width - 45;
    int bar_y = 20;
    int bar_w = 30;
    int bar_h = fb_height - 50;
    
    if (bar_x < 0 || bar_y < 0 || bar_w <= 0 || bar_h <= 0) 
        return;
    
    /* 绘制色标渐变 */
    for (int i = 0; i < bar_h; i++) {
        float t = 1.0f - (float)i / (float)bar_h;
        int16_t temp_v = (int16_t)(temp_min + t * (temp_max - temp_min));
        uint16_t color = temp_to_rgb565(temp_v);
        
        int y = bar_y + i;
        if (y < 0 || y >= fb_height) continue;
        
        for (int x = bar_x; x < bar_x + bar_w && x < fb_width; x++) {
            fb_base[y * fb_width + x] = color;
        }
    }
    
    /* 绘制色标边框 (白色) */
    uint16_t border_color = 0xFFFF;
    int i;
    
    /* 上下边框 */
    for (i = 0; i < bar_w; i++) {
        if (bar_x + i >= 0 && bar_x + i < fb_width) {
            if (bar_y >= 0 && bar_y < fb_height)
                fb_base[bar_y * fb_width + bar_x + i] = border_color;
            if (bar_y + bar_h - 1 >= 0 && bar_y + bar_h - 1 < fb_height)
                fb_base[(bar_y + bar_h - 1) * fb_width + bar_x + i] = border_color;
        }
    }
    
    /* 左右边框 */
    for (i = 0; i < bar_h; i++) {
        if (bar_y + i >= 0 && bar_y + i < fb_height) {
            if (bar_x >= 0 && bar_x < fb_width)
                fb_base[(bar_y + i) * fb_width + bar_x] = border_color;
            if (bar_x + bar_w - 1 >= 0 && bar_x + bar_w - 1 < fb_width)
                fb_base[(bar_y + i) * fb_width + bar_x + bar_w - 1] = border_color;
        }
    }
}

/* 绘制 OSD 信息 (温度范围 + FPS) */
static void draw_osd_info(void) {
    char buf[128];
    int x, y;
    int ox = 10;
    int oy = 10;
    uint16_t text_color = 0xFFFF;  /* 白色 */
    
    /* 显示环境温度 */
    snprintf(buf, sizeof(buf), "Amb: %.2fC", ambient_temp / 100.0f);
    if (g_verbose) printf("[OSD] %s\n", buf);
    
    /* 显示最小温度 */
    snprintf(buf, sizeof(buf), "Tmin: %.2fC", temp_min / 100.0f);
    
    /* 显示最大温度 */
    snprintf(buf, sizeof(buf), "Tmax: %.2fC", temp_max / 100.0f);
    
    /* 显示FPS */
    snprintf(buf, sizeof(buf), "FPS: %.1f", current_fps);
}

/* 更新温度范围统计 */
static void update_temp_range(void) {
    int16_t tmin = temp_data[0];
    int16_t tmax = temp_data[0];
    
    for (int i = 1; i < MLX90640_PIXELS; i++) {
        if (temp_data[i] < tmin) tmin = temp_data[i];
        if (temp_data[i] > tmax) tmax = temp_data[i];
    }
    
    if (g_use_auto_scale) {
        temp_min = tmin;
        temp_max = tmax;
    }
}

/* 计算 FPS */
static void update_fps(void) {
    static int count = 0;
    struct timespec now;
    
    count++;
    if (count == 1) {
        clock_gettime(CLOCK_MONOTONIC, &fps_start);
        return;
    }
    
    clock_gettime(CLOCK_MONOTONIC, &now);
    float elapsed = (now.tv_sec - fps_start.tv_sec) + 
                    (now.tv_nsec - fps_start.tv_nsec) / 1e9f;
    
    if (elapsed >= 2.0f) {
        current_fps = count / elapsed;
        count = 0;
        if (g_verbose) {
            printf("[FPS] %.1f frames/sec\n", current_fps);
        }
    }
}

/* 打印使用说明 */
static void print_usage(const char *prog) {
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  MLX90640 热成像显示程序 v2.0                             ║\n");
    printf("║  Thermal Imaging Display Application                      ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    printf("\n用法:\n");
    printf("  %s [选项]\n\n", prog);
    printf("选项:\n");
    printf("  (无)           自动缩放温度范围 (推荐)\n");
    printf("  --fixed        固定温度范围 -40°C ~ 305°C\n");
    printf("  --verbose      显示详细调试信息\n");
    printf("  --help         显示此帮助信息\n");
    printf("\n硬件配置:\n");
    printf("  传感器:  MLX90640 (32x24像素, I2C地址0x33)\n");
    printf("  温度单位: 0.01°C (int16_t格式)\n");
    printf("  屏幕:    /dev/fb0 (RGB565)\n");
    printf("\n快捷键:\n");
    printf("  Ctrl+C      退出程序\n");
    printf("\n");
}

int main(int argc, char *argv[]) {
    int mlx_fd = -1;
    uint8_t read_buf[MLX90640_FRAME_SIZE];
    int ret;
    
    /* 解析命令行参数 */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--fixed") == 0) {
            g_use_auto_scale = 0;
            printf("[CONFIG] 使用固定温度范围\n");
        } else if (strcmp(argv[i], "--verbose") == 0) {
            g_verbose = 1;
            printf("[CONFIG] 启用详细输出\n");
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            printf("[WARN] 未知选项: %s\n", argv[i]);
        }
    }
    
    printf("\n═════════════════════════════════════════════════════════════\n");
    printf("  MLX90640 热成像显示程序 v2.0\n");
    printf("═════════════════════════════════════════════════════════════\n");
    printf("[INFO] 传感器: 32x24 像素\n");
    printf("[INFO] 温度范围: -40°C ~ 305°C\n");
    printf("[INFO] 数据单位: 0.01°C (int16_t)\n");
    printf("[INFO] 读取路径: %s\n", MLX90640_DEV);
    printf("[INFO] 显示路径: %s\n", FB_DEV);
    
    /* 注册信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* 打开 MLX90640 设备 */
    printf("\n[INIT] 初始化 MLX90640 设备...\n");
    mlx_fd = open(MLX90640_DEV, O_RDONLY);
    if (mlx_fd < 0) {
        printf("[ERROR] 无法打开 %s\n", MLX90640_DEV);
        printf("[ERROR] 错误代码: %d (%s)\n", errno, strerror(errno));
        printf("[HINT] 请确保驱动已加载: insmod mlx90640_drv.ko\n");
        printf("[HINT] 设备节点已创建: /dev/mlx90640\n");
        return 1;
    }
    printf("[OK] 已打开 %s\n", MLX90640_DEV);
    
    /* 初始化 framebuffer */
    printf("\n[INIT] 初始化 Framebuffer...\n");
    if (fb_init() < 0) {
        printf("[ERROR] Framebuffer初始化失败\n");
        close(mlx_fd);
        return 1;
    }
    printf("[OK] Framebuffer: %dx%d RGB565\n", fb_width, fb_height);
    
    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║  开始显示热成像...                                         ║\n");
    printf("║  按 Ctrl+C 退出                                            ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");
    
    /* 主循环 */
    while (g_running) {
        ret = read(mlx_fd, read_buf, MLX90640_FRAME_SIZE);
        if (ret != MLX90640_FRAME_SIZE) {
            if (g_verbose) {
                printf("[WARN] 读取数据大小不匹配: %d != %d (errno: %d)\n", 
                       ret, MLX90640_FRAME_SIZE, errno);
            }
            usleep(50000);
            continue;
        }
        
        /* 解析数据: 2字节环境温度 + 768像素 × 2字节 */
        ambient_temp = (int16_t)(read_buf[0] | (read_buf[1] << 8));
        
        for (int i = 0; i < MLX90640_PIXELS; i++) {
            int offset = 2 + i * 2;
            temp_data[i] = (int16_t)(read_buf[offset] | (read_buf[offset + 1] << 8));
        }
        
        /* 更新统计 */
        update_temp_range();
        update_fps();
        
        /* 每10帧打印一次统计信息 */
        if (frame_count % 10 == 0) {
            printf("[FRAME] #%u | Amb=%.2f°C | Tmin=%.2f°C | Tmax=%.2f°C | FPS=%.1f\n",
                   frame_count, 
                   ambient_temp / 100.0f,
                   temp_min / 100.0f, 
                   temp_max / 100.0f, 
                   current_fps);
        }
        
        /* 渲染画面 */
        draw_thermal_image();
        draw_colorbar();
        draw_osd_info();
        
        frame_count++;
        usleep(50000);  /* 约 20 FPS 主循环 */
    }
    
    printf("\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║  清理中...                                               ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");
    
cleanup:
    if (mlx_fd >= 0) {
        close(mlx_fd);
        printf("[CLEANUP] 已关闭 MLX90640 设备\n");
    }
    fb_cleanup();
    printf("[CLEANUP] 已清理 Framebuffer\n");
    printf("[STATS] 总共显示 %u 帧, 平均 FPS: %.1f\n", 
           frame_count, current_fps);
    printf("[EXIT] 程序正常退出\n");
    
    return 0;
}
