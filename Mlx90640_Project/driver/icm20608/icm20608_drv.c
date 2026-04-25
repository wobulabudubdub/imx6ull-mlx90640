#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define ICM20608_CNT    1
#define ICM20608_NAME   "icm20608"
#define ICM20608_DATA_LEN  14  /* 6(accel) + 2(temp) + 6(gyro) */

/* 寄存器地址 */
#define ICM20_WHOAMI         0x75
#define ICM20_PWR_MGMT_1    0x6B
#define ICM20_ACCEL_XOUT_H  0x3B

/* 私有数据结构体 */
struct icm20608_dev {
    struct spi_device *spi;
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
};

static struct icm20608_dev *icm20608;

/* SPI 读取单个寄存器 */
static u8 icm20608_read_reg(struct spi_device *spi, u8 reg)
{
    u8 tx_data = reg | 0x80;
    u8 rx_data = 0;

    spi_write_then_read(spi, &tx_data, 1, &rx_data, 1);
    return rx_data;
}

/* SPI 写入单个寄存器 */
static void icm20608_write_reg(struct spi_device *spi, u8 reg, u8 value)
{
    u8 tx_data[2];
    tx_data[0] = reg & ~0x80;
    tx_data[1] = value;
    spi_write(spi, tx_data, 2);
}

/* SPI 连续读取多个寄存器（从 reg 开始读 len 个字节） */
static int icm20608_read_regs(struct spi_device *spi, u8 reg, u8 *buf, u8 len)
{
    u8 tx_data = reg | 0x80;

    return spi_write_then_read(spi, &tx_data, 1, buf, len);
}

/* ======== 文件操作 ======== */

static int icm20608_open(struct inode *inode, struct file *filp)
{
    filp->private_data = icm20608;
    return 0;
}

static int icm20608_release(struct inode *inode, struct file *filp)
{
    return 0;
}

/*
 * read 返回 14 字节原始数据：
 * [0-5]  Accel X/Y/Z (高字节在前, 低字节在后, 共6字节)
 * [6-7]  Temperature (高字节在前, 低字节在后, 共2字节)
 * [8-13] Gyro  X/Y/Z (高字节在前, 低字节在后, 共6字节)
 */
static ssize_t icm20608_read(struct file *filp, char __user *buf,
                              size_t count, loff_t *off)
{
    struct icm20608_dev *dev = filp->private_data;
    u8 data[ICM20608_DATA_LEN];
    int ret;

    /* 从 0x3B 连续读取 14 个寄存器 */
    ret = icm20608_read_regs(dev->spi, ICM20_ACCEL_XOUT_H, data, ICM20608_DATA_LEN);
    if (ret < 0) {
        printk(KERN_ERR "icm20608: SPI read failed\n");
        return ret;
    }

    /* 限制用户空间读取长度 */
    if (count > ICM20608_DATA_LEN)
        count = ICM20608_DATA_LEN;

    /* 拷贝到用户空间 */
    if (copy_to_user(buf, data, count)) {
        printk(KERN_ERR "icm20608: copy_to_user failed\n");
        return -EFAULT;
    }

    return count;
}

static const struct file_operations icm20608_fops = {
    .owner = THIS_MODULE,
    .open  = icm20608_open,
    .release = icm20608_release,
    .read  = icm20608_read,
};

/* ======== Probe / Remove ======== */

static int icm20608_probe(struct spi_device *spi)
{
    u8 chip_id;
    int ret;

    printk(KERN_INFO "--- ICM20608 Probe Started ---\n");

    /* 分配私有数据 */
    icm20608 = kzalloc(sizeof(*icm20608), GFP_KERNEL);
    if (!icm20608)
        return -ENOMEM;

    icm20608->spi = spi;
    spi_set_drvdata(spi, icm20608);

    /* 配置 SPI */
    spi->mode = SPI_MODE_0;
    spi->max_speed_hz = 1000000;
    spi_setup(spi);

    /* 唤醒芯片 */
    printk(KERN_INFO "=> Waking up ICM20608...\n");
    icm20608_write_reg(spi, ICM20_PWR_MGMT_1, 0x80);
    mdelay(50);
    icm20608_write_reg(spi, ICM20_PWR_MGMT_1, 0x01);
    mdelay(50);

    /* 验证芯片 ID */
    chip_id = icm20608_read_reg(spi, ICM20_WHOAMI);
    printk(KERN_INFO "=> WHO_AM_I = 0x%02X\n", chip_id);

    if (chip_id != 0xAF && chip_id != 0xAE) {
        printk(KERN_ERR "=> [ERROR] Invalid Chip ID!\n");
        ret = -ENODEV;
        goto err_free;
    }
    printk(KERN_INFO "=> [SUCCESS] ICM20608 detected!\n");

    /* ---- 注册字符设备 ---- */

    /* 1. 分配设备号 */
    ret = alloc_chrdev_region(&icm20608->devid, 0, ICM20608_CNT, ICM20608_NAME);
    if (ret < 0) {
        printk(KERN_ERR "alloc_chrdev_region failed\n");
        goto err_free;
    }
    printk(KERN_INFO "=> Major=%d, Minor=%d\n",
           MAJOR(icm20608->devid), MINOR(icm20608->devid));

    /* 2. 初始化并添加 cdev */
    cdev_init(&icm20608->cdev, &icm20608_fops);
    icm20608->cdev.owner = THIS_MODULE;
    ret = cdev_add(&icm20608->cdev, icm20608->devid, ICM20608_CNT);
    if (ret < 0) {
        printk(KERN_ERR "cdev_add failed\n");
        goto err_unreg_chrdev;
    }

    /* 3. 创建类 */
    icm20608->class = class_create(THIS_MODULE, ICM20608_NAME);
    if (IS_ERR(icm20608->class)) {
        ret = PTR_ERR(icm20608->class);
        printk(KERN_ERR "class_create failed\n");
        goto err_cdev_del;
    }

    /* 4. 创建设备节点 -> /dev/icm20608 */
    icm20608->device = device_create(icm20608->class, NULL,
                                      icm20608->devid, NULL, ICM20608_NAME);
    if (IS_ERR(icm20608->device)) {
        ret = PTR_ERR(icm20608->device);
        printk(KERN_ERR "device_create failed\n");
        goto err_class_destroy;
    }

    printk(KERN_INFO "=> /dev/%s created\n", ICM20608_NAME);
    return 0;

err_class_destroy:
    class_destroy(icm20608->class);
err_cdev_del:
    cdev_del(&icm20608->cdev);
err_unreg_chrdev:
    unregister_chrdev_region(icm20608->devid, ICM20608_CNT);
err_free:
    kfree(icm20608);
    return ret;
}

static int icm20608_remove(struct spi_device *spi)
{
    struct icm20608_dev *dev = spi_get_drvdata(spi);

    device_destroy(dev->class, dev->devid);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devid, ICM20608_CNT);
    kfree(dev);

    printk(KERN_INFO "=> ICM20608 driver removed\n");
    return 0;
}

/* ======== SPI 驱动注册 ======== */

static const struct spi_device_id icm20608_id[] = {
    {"icm20608", 0},
    {}
};
MODULE_DEVICE_TABLE(spi, icm20608_id);

static struct spi_driver icm20608_driver = {
    .driver = {
        .name = ICM20608_NAME,
        .owner = THIS_MODULE,
    },
    .probe  = icm20608_probe,
    .remove = icm20608_remove,
    .id_table = icm20608_id,
};

module_spi_driver(icm20608_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Feifei");
