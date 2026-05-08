#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xfa985410, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xab5c8cb5, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0x19bdcb34, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0xfa2a45e, __VMLINUX_SYMBOL_STR(__memzero) },
	{ 0xfbc74f64, __VMLINUX_SYMBOL_STR(__copy_from_user) },
	{ 0x67c2fa54, __VMLINUX_SYMBOL_STR(__copy_to_user) },
	{ 0x49fcab7e, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x12a38747, __VMLINUX_SYMBOL_STR(usleep_range) },
	{ 0x7267eeee, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x208614a6, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x63961a83, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0xd245d5c6, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0x3c8c7d13, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0xfeb25d8b, __VMLINUX_SYMBOL_STR(cdev_add) },
	{ 0xb6828306, __VMLINUX_SYMBOL_STR(cdev_init) },
	{ 0x29537c9e, __VMLINUX_SYMBOL_STR(alloc_chrdev_region) },
	{ 0xf3bb59b5, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0xcca6874c, __VMLINUX_SYMBOL_STR(kmem_cache_alloc) },
	{ 0x6796d347, __VMLINUX_SYMBOL_STR(i2c_master_send) },
	{ 0x243f207d, __VMLINUX_SYMBOL_STR(i2c_transfer) },
	{ 0x9d2c1f73, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x7485e15e, __VMLINUX_SYMBOL_STR(unregister_chrdev_region) },
	{ 0xb37da9a1, __VMLINUX_SYMBOL_STR(cdev_del) },
	{ 0x495be299, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0xc7a1f54a, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0xefd6cf06, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr0) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("of:N*T*Cmelexis,mlx90640*");
MODULE_ALIAS("i2c:mlx90640");

MODULE_INFO(srcversion, "2438F592E7D0B5349FD7F8A");
