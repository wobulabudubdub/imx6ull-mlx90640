/*
 * mlx90640_raw_drv.c - MLX90640 纯净版高速数据驱动
 *
 * 核心理念:
 * 内核只负责 I2C 数据搬运，不做任何浮点运算和温度换算！
 * 
 * 用户空间接口:
 * - read(): 阻塞读取一帧(Subpage)的原始 RAM 数据，固定返回 1664 字节 (832 words)
 * - ioctl(): 提供获取 EEPROM 和设置帧率的接口
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

/* ========== IOCTL 命令定义 ========== */
#define MLX90640_IOC_MAGIC 'M'
/* 获取 832 个 word 的 EEPROM 原始数据 (1664 字节) */
#define MLX90640_IOC_GET_EEPROM _IOR(MLX90640_IOC_MAGIC, 0x01, unsigned short[832])
/* 设置刷新率 (0~7) */
#define MLX90640_IOC_SET_RATE   _IOW(MLX90640_IOC_MAGIC, 0x02, int)

/* ========== 基本参数 ========== */
#define MLX90640_NAME        "mlx90640"
#define MLX90640_CNT         1
#define MLX90640_FRAME_WORDS 832
#define MLX90640_FRAME_BYTES (MLX90640_FRAME_WORDS * 2)

#define MLX90640_REG_RAM     0x0400
#define MLX90640_REG_EEPROM  0x2400
#define MLX90640_REG_STATUS  0x8000
#define MLX90640_REG_CTRL1   0x800D

#define MLX90640_STATUS_DATA_READY 0x0008

struct mlx90640_dev {
    struct i2c_client *client;
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct mutex lock;
};

static struct mlx90640_dev *mlx_dev;

/* ========== 核心 I2C 读写 ========== */

static int mlx90640_read_reg16(struct i2c_client *client, u16 reg, u16 *val)
{
    u8 addr_buf[2] = { reg >> 8, reg & 0xFF };
    u8 data_buf[2];
    struct i2c_msg msgs[2] = {
        { .addr = client->addr, .flags = 0, .len = 2, .buf = addr_buf },
        { .addr = client->addr, .flags = I2C_M_RD, .len = 2, .buf = data_buf }
    };
    int ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret == 2) {
        *val = (data_buf[0] << 8) | data_buf[1];
        return 0;
    }
    return -EIO;
}

static int mlx90640_write_reg16(struct i2c_client *client, u16 reg, u16 val)
{
    u8 buf[4] = { reg >> 8, reg & 0xFF, val >> 8, val & 0xFF };
    if (i2c_master_send(client, buf, 4) == 4) return 0;
    return -EIO;
}

/* 高效分块读取 (保留你的优秀设计，这对 i.MX6ULL 极其重要) */
static int mlx90640_read_block(struct i2c_client *client, u16 reg, u16 *buf, u16 len)
{
    u16 chunk_words = 32; 
    u16 words_read = 0;
    u8 addr_buf[2];
    u8 *data_buf = kmalloc(chunk_words * 2, GFP_KERNEL);
    
    if (!data_buf) return -ENOMEM;

    while (words_read < len) {
        u16 current_read = min_t(u16, len - words_read, chunk_words);
        u16 current_reg = reg + words_read;
        struct i2c_msg msgs[2];
        int ret, i;

        addr_buf[0] = current_reg >> 8;
        addr_buf[1] = current_reg & 0xFF;

        msgs[0].addr = client->addr; msgs[0].flags = 0; msgs[0].len = 2; msgs[0].buf = addr_buf;
        msgs[1].addr = client->addr; msgs[1].flags = I2C_M_RD; msgs[1].len = current_read * 2; msgs[1].buf = data_buf;

        ret = i2c_transfer(client->adapter, msgs, 2);
        if (ret != 2) {
            kfree(data_buf);
            return -EIO;
        }

        for (i = 0; i < current_read; i++) {
            buf[words_read + i] = (data_buf[i * 2] << 8) | data_buf[i * 2 + 1];
        }
        words_read += current_read;
    }
    kfree(data_buf);
    return 0;
}

/* ========== 文件操作接口 ========== */

static int mlx90640_open(struct inode *inode, struct file *filp)
{
    filp->private_data = mlx_dev;
    return 0;
}

/* 核心：只做一件事，把传感器当前准备好的 Raw RAM 推给应用层 */
static ssize_t mlx90640_read(struct file *filp, char __user *buf, size_t count, loff_t *off)
{
    struct mlx90640_dev *dev = filp->private_data;
    u16 *raw_ram;
    u16 status;
    int ret, timeout = 100; /* 最大等待 100 * 5ms = 500ms */

    if (count < MLX90640_FRAME_BYTES) return -EINVAL;

    raw_ram = kzalloc(MLX90640_FRAME_BYTES, GFP_KERNEL);
    if (!raw_ram) return -ENOMEM;

    mutex_lock(&dev->lock);

    /* 1. 等待数据就绪 */
    while (timeout--) {
        mlx90640_read_reg16(dev->client, MLX90640_REG_STATUS, &status);
        if (status & MLX90640_STATUS_DATA_READY) break;
        usleep_range(2000, 5000);
    }
    if (timeout < 0) {
        ret = -ETIMEDOUT;
        goto out;
    }

    /* 2. 读取整块 RAM */
    ret = mlx90640_read_block(dev->client, MLX90640_REG_RAM, raw_ram, MLX90640_FRAME_WORDS);
    if (ret < 0) goto out;

    /* 3. 清除中断标志，告诉硬件可以刷新下一帧了 */
    mlx90640_write_reg16(dev->client, MLX90640_REG_STATUS, status & ~MLX90640_STATUS_DATA_READY);

    /* 4. 推送到用户空间 */
    if (copy_to_user(buf, raw_ram, MLX90640_FRAME_BYTES)) {
        ret = -EFAULT;
        goto out;
    }
    
    ret = MLX90640_FRAME_BYTES; /* 返回读取的字节数 */

out:
    mutex_unlock(&dev->lock);
    kfree(raw_ram);
    return ret;
}

/* IOCTL: 提供设置控制寄存器和读取 EEPROM 的后门 */
static long mlx90640_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct mlx90640_dev *dev = filp->private_data;
    u16 *eeprom;
    u16 ctrl1;
    int rate, ret = 0;

    mutex_lock(&dev->lock);

    switch (cmd) {
    case MLX90640_IOC_GET_EEPROM:
        eeprom = kzalloc(MLX90640_FRAME_BYTES, GFP_KERNEL);
        if (!eeprom) { ret = -ENOMEM; break; }
        
        ret = mlx90640_read_block(dev->client, MLX90640_REG_EEPROM, eeprom, MLX90640_FRAME_WORDS);
        if (ret == 0) {
            if (copy_to_user((void __user *)arg, eeprom, MLX90640_FRAME_BYTES))
                ret = -EFAULT;
        }
        kfree(eeprom);
        break;

    case MLX90640_IOC_SET_RATE:
        if (copy_from_user(&rate, (int __user *)arg, sizeof(int))) {
            ret = -EFAULT; break;
        }
        mlx90640_read_reg16(dev->client, MLX90640_REG_CTRL1, &ctrl1);
        ctrl1 = (ctrl1 & 0xFC7F) | ((rate & 0x07) << 7);
        ret = mlx90640_write_reg16(dev->client, MLX90640_REG_CTRL1, ctrl1);
        break;

    default:
        ret = -EINVAL;
        break;
    }

    mutex_unlock(&dev->lock);
    return ret;
}

static const struct file_operations mlx90640_fops = {
    .owner          = THIS_MODULE,
    .open           = mlx90640_open,
    .read           = mlx90640_read,
    .unlocked_ioctl = mlx90640_ioctl,
};

/* ========== Probe & Remove ========== */

static int mlx90640_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    u16 ctrl1;

    mlx_dev = kzalloc(sizeof(*mlx_dev), GFP_KERNEL);
    if (!mlx_dev) return -ENOMEM;

    mlx_dev->client = client;
    mutex_init(&mlx_dev->lock);

    /* 硬件基本初始化：设为 8Hz, 18-bit 分辨率 */
    mlx90640_read_reg16(client, MLX90640_REG_CTRL1, &ctrl1);
    ctrl1 = (ctrl1 & 0xFC7F) | (0x03 << 7);   /* 8Hz */
    ctrl1 = (ctrl1 & 0xF3FF) | (0x02 << 10);  /* 18-bit */
    mlx90640_write_reg16(client, MLX90640_REG_CTRL1, ctrl1);

    alloc_chrdev_region(&mlx_dev->devid, 0, MLX90640_CNT, MLX90640_NAME);
    cdev_init(&mlx_dev->cdev, &mlx90640_fops);
    cdev_add(&mlx_dev->cdev, mlx_dev->devid, MLX90640_CNT);

    mlx_dev->class = class_create(THIS_MODULE, MLX90640_NAME);
    mlx_dev->device = device_create(mlx_dev->class, NULL, mlx_dev->devid, NULL, MLX90640_NAME);

    dev_info(&client->dev, "MLX90640 RAW driver probed successfully!\n");
    return 0;
}

static int mlx90640_remove(struct i2c_client *client)
{
    device_destroy(mlx_dev->class, mlx_dev->devid);
    class_destroy(mlx_dev->class);
    cdev_del(&mlx_dev->cdev);
    unregister_chrdev_region(mlx_dev->devid, MLX90640_CNT);
    kfree(mlx_dev);
    return 0;
}

static const struct of_device_id mlx90640_of_match[] = {
    { .compatible = "melexis,mlx90640" }, { }
};
MODULE_DEVICE_TABLE(of, mlx90640_of_match);

static struct i2c_driver mlx90640_driver = {
    .driver = { .name = MLX90640_NAME, .of_match_table = mlx90640_of_match, },
    .probe = mlx90640_probe,
    .remove = mlx90640_remove,
};

module_i2c_driver(mlx90640_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Feifei");
MODULE_DESCRIPTION("MLX90640 Raw Data Porter Drivers");