/*
 * mlx90640_app_debug.c - 调试版本，详细打印温度数据
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

int main(void) {
    int mlx_fd = open(MLX90640_DEV, O_RDONLY);
    if (mlx_fd < 0) {
        perror("打开MLX90640失败");
        return 1;
    }
    
    uint8_t buf[READ_FRAME_SIZE];
    int ret = read(mlx_fd, buf, READ_FRAME_SIZE);
    
    if (ret != READ_FRAME_SIZE) {
        printf("读取大小错误: %d/%d\n", ret, READ_FRAME_SIZE);
        close(mlx_fd);
        return 1;
    }
    
    printf("════════════════════════════════════════════════════════════\n");
    printf("MLX90640 原始数据分析\n");
    printf("════════════════════════════════════════════════════════════\n\n");
    
    /* 解析环境温度 */
    int16_t ambient = (int16_t)(buf[0] | (buf[1] << 8));
    printf("[环境温度] raw=0x%04X, 十进制=%d, 温度=%.2f°C\n\n", 
           (uint16_t)ambient, ambient, ambient/100.0f);
    
    /* 解析像素 */
    printf("[前16个像素温度 (单位: 0.01°C)]:\n");
    int16_t min_val = 32767;
    int16_t max_val = -32768;
    
    for (int i = 0; i < SENSOR_PIXELS; i++) {
        int offset = 2 + i * 2;
        int16_t val = (int16_t)(buf[offset] | (buf[offset + 1] << 8));
        
        if (val < min_val) min_val = val;
        if (val > max_val) max_val = val;
        
        if (i < 16) {
            printf("  pixel[%3d] = 0x%04X (%6d) = %7.2f°C\n", 
                   i, (uint16_t)val, val, val/100.0f);
        }
    }
    
    printf("\n[温度统计]:\n");
    printf("  最小值: %d (%.2f°C)\n", min_val, min_val/100.0f);
    printf("  最大值: %d (%.2f°C)\n", max_val, max_val/100.0f);
    printf("  范围:   %.2f°C\n", (max_val - min_val)/100.0f);
    
    close(mlx_fd);
    return 0;
}
