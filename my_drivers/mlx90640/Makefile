# 1. 内核源码的绝对路径 (需要改成你电脑里内核源码的实际路径)
KERNELDIR := /home/feifei/linux/kernel

# 2. 当前驱动代码所在的目录
CURRENT_PATH := $(shell pwd)

# 3. 告诉内核编译系统，我们要把 mlx90640_drv.c 编译成模块 (.ko)
obj-m := mlx90640_drv.o

# 4. 编译命令 (借助内核源码树的 Makefile 来编译我们现在的代码)
build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- modules

# 5. 清理规则
clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
