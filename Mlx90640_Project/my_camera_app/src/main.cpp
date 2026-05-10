/***************************************************************
 Copyright © ALIENTEK Co., Ltd. 1998-2021. All rights reserved.
 文件名 : v4l2_camera.cpp
 描述 : V4L2摄像头应用编程实战 (C++重构版)
 ***************************************************************/

#include <cstdio>
#include <cstdlib>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cerrno>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/fb.h>

#define FB_DEV              "/dev/fb0"      // LCD设备节点
#define FRAMEBUFFER_COUNT   3               // 帧缓冲数量

/*** 摄像头像素格式及其描述信息 ***/
struct camera_format {
    unsigned char description[32];  // 字符串描述信息
    unsigned int pixelformat;       // 像素格式
};

/*** 描述一个帧缓冲的信息 ***/
struct cam_buf_info {
    unsigned short *start;      // 帧缓冲起始地址
    unsigned long length;       // 帧缓冲长度
};

static int width;                       // LCD宽度
static int height;                      // LCD高度
static unsigned short *screen_base = nullptr; // LCD显存基地址
static int fb_fd = -1;                  // LCD设备文件描述符
static int v4l2_fd = -1;                // 摄像头设备文件描述符
static cam_buf_info buf_infos[FRAMEBUFFER_COUNT];
static camera_format cam_fmts[10];
static int frm_width, frm_height;       // 视频帧宽度和高度

static int fb_dev_init() {
    struct fb_var_screeninfo fb_var = {0};
    struct fb_fix_screeninfo fb_fix = {0};
    unsigned long screen_size;

    fb_fd = open(FB_DEV, O_RDWR);
    if (0 > fb_fd) {
        fprintf(stderr, "open error: %s: %s\n", FB_DEV, strerror(errno));
        return -1;
    }

    ioctl(fb_fd, FBIOGET_VSCREENINFO, &fb_var);
    ioctl(fb_fd, FBIOGET_FSCREENINFO, &fb_fix);

    screen_size = fb_fix.line_length * fb_var.yres;
    width = fb_var.xres;
    height = fb_var.yres;

    // 💡 C++ 修正：显式强转 void* 到 unsigned short*
    screen_base = static_cast<unsigned short *>(mmap(NULL, screen_size, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0));
    if (MAP_FAILED == reinterpret_cast<void *>(screen_base)) {
        perror("mmap error");
        close(fb_fd);
        return -1;
    }

    memset(screen_base, 0xFF, screen_size);
    return 0;
}

static int v4l2_dev_init(const char *device) {
    struct v4l2_capability cap = {0};

    v4l2_fd = open(device, O_RDWR);
    if (0 > v4l2_fd) {
        fprintf(stderr, "open error: %s: %s\n", device, strerror(errno));
        return -1;
    }

    ioctl(v4l2_fd, VIDIOC_QUERYCAP, &cap);

    if (!(V4L2_CAP_VIDEO_CAPTURE & cap.capabilities)) {
        fprintf(stderr, "Error: %s: No capture video device!\n", device);
        close(v4l2_fd);
        return -1;
    }

    return 0;
}

static void v4l2_enum_formats() {
    struct v4l2_fmtdesc fmtdesc = {0};

    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (0 == ioctl(v4l2_fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
        cam_fmts[fmtdesc.index].pixelformat = fmtdesc.pixelformat;
        // 💡 C++ 修正：strcpy 严格要求 char* 类型转换
        strcpy(reinterpret_cast<char *>(cam_fmts[fmtdesc.index].description), reinterpret_cast<const char *>(fmtdesc.description));
        fmtdesc.index++;
    }
}

static void v4l2_print_formats() {
    struct v4l2_frmsizeenum frmsize = {0};
    struct v4l2_frmivalenum frmival = {0};
    
    frmsize.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frmival.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (int i = 0; cam_fmts[i].pixelformat; i++) {
        printf("format<0x%x>, description<%s>\n", cam_fmts[i].pixelformat, cam_fmts[i].description);

        frmsize.index = 0;
        frmsize.pixel_format = cam_fmts[i].pixelformat;
        frmival.pixel_format = cam_fmts[i].pixelformat;
        while (0 == ioctl(v4l2_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)) {
            printf("size<%d*%d> ", frmsize.discrete.width, frmsize.discrete.height);
            frmsize.index++;

            frmival.index = 0;
            frmival.width = frmsize.discrete.width;
            frmival.height = frmsize.discrete.height;
            while (0 == ioctl(v4l2_fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival)) {
                printf("<%dfps>", frmival.discrete.denominator / frmival.discrete.numerator);
                frmival.index++;
            }
            printf("\n");
        }
        printf("\n");
    }
}

static int v4l2_set_format() {
    struct v4l2_format fmt = {0};
    struct v4l2_streamparm streamparm = {0};

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565; 
    if (0 > ioctl(v4l2_fd, VIDIOC_S_FMT, &fmt)) {
        fprintf(stderr, "ioctl error: VIDIOC_S_FMT: %s\n", strerror(errno));
        return -1;
    }

    if (V4L2_PIX_FMT_RGB565 != fmt.fmt.pix.pixelformat) {
        fprintf(stderr, "Error: the device does not support RGB565 format!\n");
        return -1;
    }

    frm_width = fmt.fmt.pix.width;
    frm_height = fmt.fmt.pix.height;
    printf("视频帧大小<%d * %d>\n", frm_width, frm_height);

    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(v4l2_fd, VIDIOC_G_PARM, &streamparm);

    if (V4L2_CAP_TIMEPERFRAME & streamparm.parm.capture.capability) {
        streamparm.parm.capture.timeperframe.numerator = 1;
        streamparm.parm.capture.timeperframe.denominator = 30;
        if (0 > ioctl(v4l2_fd, VIDIOC_S_PARM, &streamparm)) {
            fprintf(stderr, "ioctl error: VIDIOC_S_PARM: %s\n", strerror(errno));
            return -1;
        }
    }
    return 0;
}

static int v4l2_init_buffer() {
    struct v4l2_requestbuffers reqbuf = {0};

    reqbuf.count = FRAMEBUFFER_COUNT;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    if (0 > ioctl(v4l2_fd, VIDIOC_REQBUFS, &reqbuf)) {
        fprintf(stderr, "ioctl error: VIDIOC_REQBUFS: %s\n", strerror(errno));
        return -1;
    }

    for (int i = 0; i < FRAMEBUFFER_COUNT; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        ioctl(v4l2_fd, VIDIOC_QUERYBUF, &buf);
        buf_infos[i].length = buf.length;
        // 💡 C++ 修正：显式强转 void* 到 unsigned short*
        buf_infos[i].start = static_cast<unsigned short *>(mmap(NULL, buf.length,
                PROT_READ | PROT_WRITE, MAP_SHARED,
                v4l2_fd, buf.m.offset));
        if (MAP_FAILED == reinterpret_cast<void *>(buf_infos[i].start)) {
            perror("mmap error");
            return -1;
        }
    }

    for (int i = 0; i < FRAMEBUFFER_COUNT; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (0 > ioctl(v4l2_fd, VIDIOC_QBUF, &buf)) {
            fprintf(stderr, "ioctl error: VIDIOC_QBUF: %s\n", strerror(errno));
            return -1;
        }
    }
    return 0;
}

static int v4l2_stream_on() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 > ioctl(v4l2_fd, VIDIOC_STREAMON, &type)) {
        fprintf(stderr, "ioctl error: VIDIOC_STREAMON: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static void v4l2_read_data() {
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    int min_w = (width > frm_width) ? frm_width : width;
    int min_h = (height > frm_height) ? frm_height : height;

    while (true) {
        for(int i = 0; i < FRAMEBUFFER_COUNT; i++) {
            buf.index = i;
            ioctl(v4l2_fd, VIDIOC_DQBUF, &buf); // 出队

            unsigned short *base = screen_base;
            unsigned short *start = buf_infos[buf.index].start;
            for (int j = 0; j < min_h; j++) {
                memcpy(base, start, min_w * 2);
                base += width;
                start += frm_width;
            }

            ioctl(v4l2_fd, VIDIOC_QBUF, &buf); // 入队
        }
    }
}

int main(int argc, char *argv[]) {
    if (2 != argc) {
        fprintf(stderr, "Usage: %s <video_dev>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    if (fb_dev_init()) exit(EXIT_FAILURE);
    if (v4l2_dev_init(argv[1])) exit(EXIT_FAILURE);

    v4l2_enum_formats();
    v4l2_print_formats();

    if (v4l2_set_format()) exit(EXIT_FAILURE);
    if (v4l2_init_buffer()) exit(EXIT_FAILURE);
    if (v4l2_stream_on()) exit(EXIT_FAILURE);

    v4l2_read_data();

    exit(EXIT_SUCCESS);
}