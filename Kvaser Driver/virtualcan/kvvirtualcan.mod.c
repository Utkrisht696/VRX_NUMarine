#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x6b14f351, "vCanAddCardChannel" },
	{ 0x30372d96, "queue_release" },
	{ 0x4096d7f9, "vCanFlushSendBuffer" },
	{ 0x825e4adf, "vCanInit" },
	{ 0x37a0cba, "kfree" },
	{ 0x833811ec, "vCanGetCardInfo" },
	{ 0xa0eeba40, "pcpu_hot" },
	{ 0xe2964344, "__wake_up" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0x122c3a7e, "_printk" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xbde0616d, "vCanGetCardInfo2" },
	{ 0xadc10e64, "vCanInitData" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0xf6149df4, "set_capability_value" },
	{ 0x9ed12e20, "kmalloc_large" },
	{ 0x265917c7, "vCanDispatchEvent" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0xfb578fc5, "memset" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x679e43d1, "queue_empty" },
	{ 0x1905693e, "vCanRemoveCardChannel" },
	{ 0x7e8ceea, "vCanTime" },
	{ 0xb236c6eb, "kmalloc_trace" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x6438f12, "vCanCleanup" },
	{ 0x1a103589, "kmalloc_caches" },
	{ 0x43581fb4, "module_layout" },
};

MODULE_INFO(depends, "kvcommon");


MODULE_INFO(srcversion, "F7FA974643FF34FEB47B347");
