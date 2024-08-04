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
	{ 0x7939ca3d, "usb_alloc_urb" },
	{ 0xcd231725, "try_module_get" },
	{ 0x49cd25ed, "alloc_workqueue" },
	{ 0xb33ded93, "usb_free_urb" },
	{ 0x4a3ad70e, "wait_for_completion_timeout" },
	{ 0x6b14f351, "vCanAddCardChannel" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0xb79c47fe, "usb_alloc_coherent" },
	{ 0xa6257a2f, "complete" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x30372d96, "queue_release" },
	{ 0x55555880, "queue_reinit" },
	{ 0x244ab863, "queue_back" },
	{ 0x608741b5, "__init_swait_queue_head" },
	{ 0x10fa71db, "queue_remove_wait_for_space" },
	{ 0x5b8de061, "usb_register_driver" },
	{ 0x4096d7f9, "vCanFlushSendBuffer" },
	{ 0x825e4adf, "vCanInit" },
	{ 0x37a0cba, "kfree" },
	{ 0x833811ec, "vCanGetCardInfo" },
	{ 0xa0eeba40, "pcpu_hot" },
	{ 0xe2964344, "__wake_up" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x37e5a7ae, "wake_up_process" },
	{ 0x122c3a7e, "_printk" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xfd85a8dc, "usb_bulk_msg" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xbde0616d, "vCanGetCardInfo2" },
	{ 0x975ba36c, "usb_submit_urb" },
	{ 0xadc10e64, "vCanInitData" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0xa1ea6db, "usb_free_coherent" },
	{ 0x1146ef24, "__module_put_and_kthread_exit" },
	{ 0x8c03d20c, "destroy_workqueue" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0xf6149df4, "set_capability_value" },
	{ 0x9ed12e20, "kmalloc_large" },
	{ 0x90f1996f, "usb_deregister" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x265917c7, "vCanDispatchEvent" },
	{ 0x87d7787f, "queue_add_wait_for_space" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0xfb578fc5, "memset" },
	{ 0x25974000, "wait_for_completion" },
	{ 0x9166fc03, "__flush_workqueue" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x6782eeca, "queue_push" },
	{ 0x679e43d1, "queue_empty" },
	{ 0xaad8c7d6, "default_wake_function" },
	{ 0x1905693e, "vCanRemoveCardChannel" },
	{ 0x9cef2f25, "kthread_create_on_node" },
	{ 0xaa3abd68, "usb_kill_urb" },
	{ 0x56470118, "__warn_printk" },
	{ 0xa02aea3a, "queue_length" },
	{ 0xb236c6eb, "kmalloc_trace" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x6438f12, "vCanCleanup" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xfe2fd6f8, "queue_init" },
	{ 0x1a103589, "kmalloc_caches" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x43581fb4, "module_layout" },
};

MODULE_INFO(depends, "kvcommon");

MODULE_ALIAS("usb:v0BFDp0004d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0002d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0005d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0003d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "CE3AE95F4D4633BEDF33596");
