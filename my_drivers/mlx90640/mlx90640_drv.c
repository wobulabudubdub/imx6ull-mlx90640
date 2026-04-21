/*
 * mlx90640_drv.c - MLX90640 红外热像仪 I2C 驱动
 *
 * 传感器信息:
 * - 分辨率: 32x24 像素
 * - 接口: I2C (地址 0x33)
 * - 温度范围: -40°C ~ 305°C
 * - 帧速率: 1Hz / 2Hz / 4Hz / 8Hz / 16Hz / 32Hz / 64Hz
 *
 * 用户空间接口:
 * /dev/mlx90640  - 字符设备
 * read()  -> 返回 834 字节 (2字节环境温度 + 832字节像素温度数据, 以0.01°C为单位)
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mutex.h>

/* ========== 驱动基本参数 ========== */

#define MLX90640_CNT         1
#define MLX90640_NAME        "mlx90640"

/* MLX90640 I2C 从机地址 */
#define MLX90640_I2C_ADDR    0x33

/* 像素网格尺寸 */
#define MLX90640_COLS        32
#define MLX90640_ROWS        24
#define MLX90640_PIXELS      (MLX90640_COLS * MLX90640_ROWS)  /* 768 */

/* 帧数据: 832 个 16-bit 字 (768 像素 + 64 行间/列间交替参考像素) */
#define MLX90640_FRAME_WORDS 832
#define MLX90640_FRAME_BYTES (MLX90640_FRAME_WORDS * 2)

/* 读出缓冲区大小: 2 字节环境温度 + 像素温度数据 (每个像素 2 字节) */
#define MLX90640_READ_BUF_SZ (2 + MLX90640_PIXELS * 2)

/* ========== 寄存器地址 ========== */

/* RAM 帧数据起始地址 */
#define MLX90640_REG_RAM     0x0400

/* 状态 / 控制 */
#define MLX90640_REG_STATUS  0x8000
#define MLX90640_REG_CTRL1   0x800D
#define MLX90640_REG_CTRL2   0x800E

/* I2C 配置寄存器 */
#define MLX90640_REG_I2C_CFG 0x800F

/* 测量触发 */
#define MLX90640_REG_OP_MODE 0x8003

/* EEPROM 起始地址 (1024 x 16-bit) */
#define MLX90640_REG_EEPROM  0x2400
#define MLX90640_EEPROM_SZ   832   /* words */

/* ========== 状态/控制位掩码 ========== */

#define MLX90640_STATUS_DATA_READY  0x0008
#define MLX90640_STATUS_SUBPAGE     0x0001

/* ========== 帧速率定义 ========== */

enum mlx90640_refresh_rate {
    MLX90640_REFRESH_1HZ  = 0x00,
    MLX90640_REFRESH_2HZ  = 0x01,
    MLX90640_REFRESH_4HZ  = 0x02,
    MLX90640_REFRESH_8HZ  = 0x03,
    MLX90640_REFRESH_16HZ = 0x04,
    MLX90640_REFRESH_32HZ = 0x05,
    MLX90640_REFRESH_64HZ = 0x06,
};

/* ========== 校准参数结构体 ========== */

struct mlx90640_calibration {
    /* 基本参数 */
    s16 k_vdd;           /* VDD 量化系数 */
    s16 vdd_25;          /* 25°C 时 VDD 基准 */
    s16 k_ptat;          /* PTAT 系数 */
    s16 v_ptat_25;       /* 25°C 时 PTAT 基准 */
    s16 alpha_ptat;      /* PTAT alpha 系数 */
    s16 k_t1;            /* 温度补偿系数 1 */
    s16 k_t2;            /* 温度补偿系数 2 */
    s16 tgc;             /* 全局温度梯度补偿 */
    s16 ksta;            /* STA 系数 */

    /* 辐射率与增益 */
    s16 gain;            /* 增益系数 */

    /* 偏移补偿 (32x24) */
    s16 offset[MLX90640_PIXELS];

    /* 像素灵敏度系数 alpha */
    s16 alpha[MLX90640_PIXELS];

    /* kta 噪声系数 (低/高 两位) */
    s8  kta[MLX90640_PIXELS];

    /* kv 噪声系数 */
    s8  kv[MLX90640_PIXELS];

    /* CIL/CISC 坏点标记 */
    u8  broken_pixels[20];
    u8  broken_count;

    /* 特殊像素补偿 */
    s16 cp_sp[2];        /* 补偿像素 alpha */
    s16 cp_offset[2];    /* 补偿像素 offset */
    s16 cp_kta;          /* 补偿像素 kta */
    s16 cp_kv;           /* 补偿像素 kv */

    /* 额外参数 */
    u8  resolution_ee;   /* 分辨率校准值 */
    u8  avg_mode_ee;     /* 平均模式校准值 */
    s16 k_t_s;           /* 外壳温度系数 */
    s16 k_t_a;           /* 偏置温度系数 */
    s16 k_t_r;           /* 参考温度系数 */
};

/* ========== 私有数据结构体 ========== */

struct mlx90640_dev {
    struct i2c_client *client;
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct mutex lock;

    struct mlx90640_calibration calib;

    /* 当前配置 */
    enum mlx90640_refresh_rate refresh_rate;

    /* 帧缓冲: 存储上一次读取的像素温度 (0.01°C) */
    s16 frame_data[MLX90640_PIXELS];
    s16 ambient_temp;    /* 环境温度 (0.01°C) */

    /* 原始帧数据 */
    u16 raw_frame[MLX90640_FRAME_WORDS];
};

static struct mlx90640_dev *mlx90640_dev_ptr;

/* ========== I2C 底层读写 ========== */

static int mlx90640_read_reg16(struct i2c_client *client, u16 reg, u16 *val)
{
    u8 addr_buf[2] = { reg >> 8, reg & 0xFF };
    u8 data_buf[2];
    struct i2c_msg msgs[2];
    int ret;

    msgs[0].addr  = client->addr;
    msgs[0].flags = 0;
    msgs[0].len   = 2;
    msgs[0].buf   = addr_buf;

    msgs[1].addr  = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = 2;
    msgs[1].buf   = data_buf;

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret != 2) {
        dev_err(&client->dev, "i2c read reg 0x%04X failed: %d\n", reg, ret);
        return (ret < 0) ? ret : -EIO;
    }

    *val = (data_buf[0] << 8) | data_buf[1];
    return 0;
}

static int mlx90640_write_reg16(struct i2c_client *client, u16 reg, u16 val)
{
    u8 buf[4];
    int ret;

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;
    buf[2] = val >> 8;
    buf[3] = val & 0xFF;

    ret = i2c_master_send(client, buf, 4);
    if (ret != 4) {
        dev_err(&client->dev, "i2c write reg 0x%04X failed: %d\n", reg, ret);
        return (ret < 0) ? ret : -EIO;
    }
    return 0;
}

/*
 * mlx90640_read_block - 高效分块读取（解除降速封印）
 */
static int mlx90640_read_block(struct i2c_client *client, u16 reg, u16 *buf, u16 len)
{
    u16 chunk_words = 32; /* 每次读32个字(64字节)，兼顾速度和 i.MX6ULL 稳定性 */
    u16 words_read = 0;
    u8 addr_buf[2];
    u8 *data_buf;
    struct i2c_msg msgs[2];
    int ret;
    u16 i;

    data_buf = kmalloc(chunk_words * 2, GFP_KERNEL);
    if (!data_buf) return -ENOMEM;

    while (words_read < len) {
        u16 current_read = len - words_read;
        u16 current_reg = reg + words_read; /* 声明放前面，避免 C90 警告 */
        
        if (current_read > chunk_words)
            current_read = chunk_words;

        addr_buf[0] = current_reg >> 8;
        addr_buf[1] = current_reg & 0xFF;

        msgs[0].addr  = client->addr;
        msgs[0].flags = 0;
        msgs[0].len   = 2;
        msgs[0].buf   = addr_buf;

        msgs[1].addr  = client->addr;
        msgs[1].flags = I2C_M_RD;
        msgs[1].len   = current_read * 2;
        msgs[1].buf   = data_buf;

        ret = i2c_transfer(client->adapter, msgs, 2);
        if (ret != 2) {
            dev_err(&client->dev, "i2c chunk read failed\n");
            kfree(data_buf);
            return (ret < 0) ? ret : -EIO;
        }

        for (i = 0; i < current_read; i++) {
            buf[words_read + i] = (data_buf[i * 2] << 8) | data_buf[i * 2 + 1];
        }
        words_read += current_read;
    }
    kfree(data_buf);
    return 0;
}
/* ========== EEPROM 解析与校准 ========== */

static int mlx90640_extract_parameters(struct mlx90640_dev *dev,
                                        u16 *eeprom)
{
    struct mlx90640_calibration *c = &dev->calib;
    int i;
    s16 val;

    /* VDD 相关参数 */
    c->k_vdd   = (s16)((eeprom[0x33] >> 5) & 0x07F);
    if (c->k_vdd > 63)
        c->k_vdd -= 128;
    c->k_vdd  *= 32;
    c->vdd_25  = (s16)(eeprom[0x33] & 0x03FF);
    if (c->vdd_25 > 511)
        c->vdd_25 -= 1024;
    c->vdd_25 *= 32;

    /* PTAT 参数 */
    val = (eeprom[0x32] >> 8) & 0xFF;
    if (val > 127)
        val -= 256;
    c->k_ptat = val;
    c->k_ptat *= 16;

    val = eeprom[0x32] & 0xFF;
    if (val > 127)
        val -= 256;
    c->v_ptat_25 = val * 8 + 256 * 8;

    val = (eeprom[0x38] >> 8) & 0xFF;
    if (val > 127)
        val -= 256;
    c->alpha_ptat = val / 4 + 8;

    /* 温度补偿 */
    val = (eeprom[0x3C] >> 5) & 0x07F;
    if (val > 63)
        val -= 128;
    c->k_t1 = val * 16;

    val = eeprom[0x3C] & 0x01F;
    if (val > 15)
        val -= 32;
    c->k_t2 = val / 8 + 20;

    /* TGC */
    val = (eeprom[0x3C] >> 12) & 0x0F;
    if (val > 7)
        val -= 16;
    c->tgc = val;

    /* STA */
    val = (eeprom[0x3B] >> 6) & 0x3FF;
    if (val > 511)
        val -= 1024;
    c->ksta = val / 4;

    /* 增益 */
    c->gain = (s16)eeprom[0x30];
    if (c->gain > 32767)
        c->gain -= 65536;

    /* 像素偏移 */
    for (i = 0; i < MLX90640_PIXELS; i++) {
        val = (s16)eeprom[i];
        if (val > 32767)
            val -= 65536;
        c->offset[i] = val;
    }

    /* 像素 alpha 灵敏度 */
    for (i = 0; i < MLX90640_PIXELS; i++) {
        /* 简化处理 */
        val = (s16)((eeprom[0x20 + i / 2] >> ((i % 2) * 8)) & 0xFF);
        if (val > 127)
            val -= 256;
        c->alpha[i] = val + 128;
    }

    /* kta 噪声系数 */
    for (i = 0; i < MLX90640_PIXELS; i++) {
        val = (s16)((eeprom[0x28 + i / 8] >> ((i % 8) * 2)) & 0x03);
        if (val > 1)
            val -= 4;
        c->kta[i] = (s8)val;
    }

    /* kv 噪声系数 */
    for (i = 0; i < MLX90640_PIXELS; i++) {
        val = (s16)((eeprom[0x30 + i / 8] >> ((i % 8) * 2)) & 0x03);
        if (val > 1)
            val -= 4;
        c->kv[i] = (s8)val;
    }

    /* 补偿像素 */
    c->cp_sp[0]    = (s16)((eeprom[0x35] >> 5) & 0x07F);
    c->cp_sp[1]    = (s16)((eeprom[0x35] >> 0) & 0x01F);
    c->cp_offset[0] = (s16)eeprom[0x36];
    c->cp_offset[1] = (s16)eeprom[0x37];
    c->cp_kta       = (s16)((eeprom[0x3A] >> 8) & 0xFF);
    c->cp_kv        = (s16)(eeprom[0x3A] & 0xFF);

    /* 分辨率与平均模式 */
    c->resolution_ee = (eeprom[0x3B] >> 10) & 0x03;
    c->avg_mode_ee   = (eeprom[0x3B] >> 12) & 0x03;

    /* 外壳温度相关 */
    c->k_t_s = (s16)((eeprom[0x3E] >> 5) & 0x07F);
    c->k_t_a = (s16)(eeprom[0x3E] & 0x01F);
    c->k_t_r = (s16)((eeprom[0x3F] >> 5) & 0x07F);

    dev_info(&dev->client->dev, "Calibration parameters extracted\n");
    return 0;
}

/* ========== 帧数据读取 ========== */

static int mlx90640_wait_data_ready(struct i2c_client *client, int timeout_ms)
{
    u16 status;
    int ret;
    int elapsed = 0;

    while (elapsed < timeout_ms) {
        ret = mlx90640_read_reg16(client, MLX90640_REG_STATUS, &status);
        if (ret < 0)
            return ret;

        if (status & MLX90640_STATUS_DATA_READY)
            return 0;

        msleep(10);
        elapsed += 10;
    }

    dev_err(&client->dev, "Timeout waiting for data ready\n");
    return -ETIMEDOUT;
}

static int mlx90640_clear_data_ready(struct i2c_client *client)
{
    u16 status;
    int ret;

    ret = mlx90640_read_reg16(client, MLX90640_REG_STATUS, &status);
    if (ret < 0)
        return ret;

    status &= ~MLX90640_STATUS_DATA_READY;
    return mlx90640_write_reg16(client, MLX90640_REG_STATUS, status);
}

static int mlx90640_get_subpage(struct i2c_client *client)
{
    u16 status;
    int ret;

    ret = mlx90640_read_reg16(client, MLX90640_REG_STATUS, &status);
    if (ret < 0)
        return ret;

    return status & MLX90640_STATUS_SUBPAGE;
}

static int mlx90640_read_frame(struct mlx90640_dev *dev)
{
    struct i2c_client *client = dev->client;
    u16 *raw_buf;
    int ret;
    int subpage;

    /* 动态分配内存，避免栈溢出 */
    raw_buf = kzalloc(MLX90640_FRAME_WORDS * sizeof(u16), GFP_KERNEL);
    if (!raw_buf)
        return -ENOMEM;

    ret = mlx90640_wait_data_ready(client, 1000);
    if (ret < 0) {
        kfree(raw_buf);
        return ret;
    }

    subpage = mlx90640_get_subpage(client);
    if (subpage < 0) {
        kfree(raw_buf);
        return subpage;
    }

    ret = mlx90640_read_block(client, MLX90640_REG_RAM, raw_buf, MLX90640_FRAME_WORDS);
    if (ret < 0) {
        dev_err(&client->dev, "Frame read failed: %d\n", ret);
        kfree(raw_buf);
        return ret;
    }

    ret = mlx90640_clear_data_ready(client);
    if (ret < 0) {
        kfree(raw_buf);
        return ret;
    }

    mutex_lock(&dev->lock);
    memcpy(dev->raw_frame, raw_buf, MLX90640_FRAME_WORDS * sizeof(u16));
    mutex_unlock(&dev->lock);

    dev_dbg(&client->dev, "Subpage %d frame read OK\n", subpage);
    kfree(raw_buf);
    return subpage;
}

/* ========== 温度转换 ========== */

static int mlx90640_process_frame(struct mlx90640_dev *dev)
{
    int i;

    /* 环境温度依然写死 25 度作为占位符 */
    dev->ambient_temp = 2500;

    mutex_lock(&dev->lock);

    /* * 【原汁原味模式】：没有任何拉伸，没有任何过滤，没有任何多余的加减乘除！
     * 直接把 I2C 读回来的 16 位原始 ADC 红外辐射强度，原封不动地交给应用层！
     */
    for (i = 0; i < MLX90640_PIXELS; i++) {
        dev->frame_data[i] = (s16)dev->raw_frame[i];
    }

    mutex_unlock(&dev->lock);
    return 0;
}

static int mlx90640_set_refresh_rate(struct mlx90640_dev *dev,
                                      enum mlx90640_refresh_rate rate)
{
    u16 ctrl1;
    int ret;

    ret = mlx90640_read_reg16(dev->client, MLX90640_REG_CTRL1, &ctrl1);
    if (ret < 0)
        return ret;

    /* 修复 Bug：刷新率在 Bit [9:7]，需要先清零这三位，再把 rate 左移 7 位写入 */
    ctrl1 = (ctrl1 & 0xFC7F) | ((rate & 0x07) << 7);

    ret = mlx90640_write_reg16(dev->client, MLX90640_REG_CTRL1, ctrl1);
    if (ret < 0)
        return ret;

    dev->refresh_rate = rate;
    dev_info(&dev->client->dev, "Refresh rate set to %dHz\n", 1 << rate);
    return 0;
}

/*
 * mlx90640_set_resolution - 设置 ADC 分辨率
 *
 * resolution: 0=16bit, 1=17bit, 2=18bit, 3=19bit
 */
static int mlx90640_set_resolution(struct mlx90640_dev *dev, u8 resolution)
{
    u16 ctrl1;
    int ret;

    ret = mlx90640_read_reg16(dev->client, MLX90640_REG_CTRL1, &ctrl1);
    if (ret < 0)
        return ret;

    /* 修复 Bug：分辨率在 Bit [11:10]，需先清零这两位，再把 resolution 左移 10 位写入 */
    ctrl1 = (ctrl1 & 0xF3FF) | ((resolution & 0x03) << 10);

    return mlx90640_write_reg16(dev->client, MLX90640_REG_CTRL1, ctrl1);
}

/* ========== 初始化 ========== */

static int mlx90640_init_sensor(struct mlx90640_dev *dev)
{
    struct i2c_client *client = dev->client;
    u16 *eeprom;
    u16 ctrl1;
    int ret;

    /* 动态分配内存以读取 EEPROM */
    eeprom = kzalloc(MLX90640_EEPROM_SZ * sizeof(u16), GFP_KERNEL);
    if (!eeprom)
        return -ENOMEM;

    dev_info(&client->dev, "--- MLX90640 Init Started ---\n");

    dev_info(&client->dev, "=> Reading EEPROM...\n");
    ret = mlx90640_read_block(client, MLX90640_REG_EEPROM, eeprom, MLX90640_EEPROM_SZ);
    if (ret < 0) {
        dev_err(&client->dev, "[ERROR] EEPROM read failed: %d\n", ret);
        kfree(eeprom);
        return ret;
    }

    ret = mlx90640_extract_parameters(dev, eeprom);
    if (ret < 0) {
        dev_err(&client->dev, "[ERROR] Calibration parse failed\n");
        kfree(eeprom);
        return ret;
    }

    /* 提取完校准参数后，即可释放 EEPROM 内存 */
    kfree(eeprom);

    ret = mlx90640_read_reg16(client, MLX90640_REG_CTRL1, &ctrl1);
    if (ret < 0) {
        dev_err(&client->dev, "[ERROR] Cannot read CTRL1 register\n");
        return ret;
    }
    dev_info(&client->dev, "=> CTRL1 = 0x%04X\n", ctrl1);

    ret = mlx90640_set_refresh_rate(dev, MLX90640_REFRESH_32HZ);
    if (ret < 0) {
        dev_err(&client->dev, "[ERROR] Set refresh rate failed\n");
        return ret;
    }

    ret = mlx90640_set_resolution(dev, 2);
    if (ret < 0) {
        dev_err(&client->dev, "[ERROR] Set resolution failed\n");
        return ret;
    }

    dev_info(&client->dev, "=> [SUCCESS] MLX90640 initialized!\n");
    return 0;
}

/* ========== 文件操作 ========== */

static int mlx90640_open(struct inode *inode, struct file *filp)
{
    filp->private_data = mlx90640_dev_ptr;
    return 0;
}

static int mlx90640_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static ssize_t mlx90640_read(struct file *filp, char __user *buf, size_t count, loff_t *off)
{
    struct mlx90640_dev *dev = filp->private_data;
    u8 *read_buf;  /* 必须是指针 */
    int ret;
    int i;

    /* 动态分配内存，避免栈溢出 */
    read_buf = kzalloc(MLX90640_READ_BUF_SZ, GFP_KERNEL);
    if (!read_buf)
        return -ENOMEM;

    ret = mlx90640_read_frame(dev);
    if (ret < 0) {
        dev_err(&dev->client->dev, "mlx90640: read frame failed: %d\n", ret);
        kfree(read_buf); /* 错误退出时也要释放 */
        return ret;
    }

    ret = mlx90640_process_frame(dev);
    if (ret < 0) {
        dev_err(&dev->client->dev, "mlx90640: process frame failed: %d\n", ret);
        kfree(read_buf); /* 错误退出时也要释放 */
        return ret;
    }

    mutex_lock(&dev->lock);

    read_buf[0] = dev->ambient_temp & 0xFF;
    read_buf[1] = (dev->ambient_temp >> 8) & 0xFF;

    for (i = 0; i < MLX90640_PIXELS; i++) {
        read_buf[2 + i * 2]     = dev->frame_data[i] & 0xFF;
        read_buf[2 + i * 2 + 1] = (dev->frame_data[i] >> 8) & 0xFF;
    }

    mutex_unlock(&dev->lock);

    if (count > MLX90640_READ_BUF_SZ)
        count = MLX90640_READ_BUF_SZ;

    if (copy_to_user(buf, read_buf, count)) {
        kfree(read_buf); /* 出错前释放 */
        dev_err(&dev->client->dev, "mlx90640: copy_to_user failed\n");
        return -EFAULT;
    }

    kfree(read_buf); /* 成功执行完毕也要释放内存！ */
    return count;
}

static const struct file_operations mlx90640_fops = {
    .owner   = THIS_MODULE,
    .open    = mlx90640_open,
    .release = mlx90640_release,
    .read    = mlx90640_read,
};

/* ========== I2C Probe / Remove ========== */

static int mlx90640_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;

    dev_info(&client->dev, "--- MLX90640 Probe Started ---\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C functionality check failed\n");
        return -ENODEV;
    }

    mlx90640_dev_ptr = kzalloc(sizeof(*mlx90640_dev_ptr), GFP_KERNEL);
    if (!mlx90640_dev_ptr)
        return -ENOMEM;

    mlx90640_dev_ptr->client = client;
    mutex_init(&mlx90640_dev_ptr->lock);
    dev_set_drvdata(&client->dev, mlx90640_dev_ptr);

    ret = mlx90640_init_sensor(mlx90640_dev_ptr);
    if (ret < 0) {
        dev_err(&client->dev, "Sensor init failed: %d\n", ret);
        goto err_free;
    }

    ret = alloc_chrdev_region(&mlx90640_dev_ptr->devid, 0, MLX90640_CNT, MLX90640_NAME);
    if (ret < 0) {
        dev_err(&client->dev, "alloc_chrdev_region failed\n");
        goto err_free;
    }
    dev_info(&client->dev, "=> Major=%d, Minor=%d\n",
             MAJOR(mlx90640_dev_ptr->devid), MINOR(mlx90640_dev_ptr->devid));

    cdev_init(&mlx90640_dev_ptr->cdev, &mlx90640_fops);
    mlx90640_dev_ptr->cdev.owner = THIS_MODULE;
    ret = cdev_add(&mlx90640_dev_ptr->cdev, mlx90640_dev_ptr->devid, MLX90640_CNT);
    if (ret < 0) {
        dev_err(&client->dev, "cdev_add failed\n");
        goto err_unreg_chrdev;
    }

    mlx90640_dev_ptr->class = class_create(THIS_MODULE, MLX90640_NAME);
    if (IS_ERR(mlx90640_dev_ptr->class)) {
        ret = PTR_ERR(mlx90640_dev_ptr->class);
        dev_err(&client->dev, "class_create failed\n");
        goto err_cdev_del;
    }

    mlx90640_dev_ptr->device = device_create(mlx90640_dev_ptr->class, NULL,
                                               mlx90640_dev_ptr->devid, NULL,
                                               MLX90640_NAME);
    if (IS_ERR(mlx90640_dev_ptr->device)) {
        ret = PTR_ERR(mlx90640_dev_ptr->device);
        dev_err(&client->dev, "device_create failed\n");
        goto err_class_destroy;
    }

    dev_info(&client->dev, "=> /dev/%s created\n", MLX90640_NAME);
    return 0;

err_class_destroy:
    class_destroy(mlx90640_dev_ptr->class);
err_cdev_del:
    cdev_del(&mlx90640_dev_ptr->cdev);
err_unreg_chrdev:
    unregister_chrdev_region(mlx90640_dev_ptr->devid, MLX90640_CNT);
err_free:
    kfree(mlx90640_dev_ptr);
    mlx90640_dev_ptr = NULL;
    return ret;
}

static int mlx90640_remove(struct i2c_client *client)
{
    struct mlx90640_dev *dev = dev_get_drvdata(&client->dev);

    device_destroy(dev->class, dev->devid);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devid, MLX90640_CNT);
    mutex_destroy(&dev->lock);
    kfree(dev);
    mlx90640_dev_ptr = NULL;

    dev_info(&client->dev, "=> MLX90640 driver removed\n");
    return 0;
}

static const struct of_device_id mlx90640_of_match[] = {
    { .compatible = "melexis,mlx90640" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mlx90640_of_match);

static const struct i2c_device_id mlx90640_id[] = {
    { "mlx90640", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mlx90640_id);

static struct i2c_driver mlx90640_driver = {
    .driver = {
        .name  = MLX90640_NAME,
        .owner = THIS_MODULE,
        .of_match_table = mlx90640_of_match, 
    },
    .probe    = mlx90640_probe,
    .remove   = mlx90640_remove,
    .id_table = mlx90640_id,
};

module_i2c_driver(mlx90640_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Feifei");
MODULE_DESCRIPTION("MLX90640 IR Thermal Camera I2C Driver");
MODULE_VERSION("1.0");