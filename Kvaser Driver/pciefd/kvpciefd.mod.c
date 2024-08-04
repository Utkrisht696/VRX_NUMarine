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
	{ 0xc1514a3b, "free_irq" },
	{ 0xc31db0ce, "is_vmalloc_addr" },
	{ 0xa78af5f3, "ioread32" },
	{ 0x4a3ad70e, "wait_for_completion_timeout" },
	{ 0x3e04513b, "pci_enable_device" },
	{ 0x4a453f53, "iowrite32" },
	{ 0x6b14f351, "vCanAddCardChannel" },
	{ 0x5b44a2d8, "pci_iomap" },
	{ 0xa6257a2f, "complete" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x30372d96, "queue_release" },
	{ 0x55555880, "queue_reinit" },
	{ 0x608741b5, "__init_swait_queue_head" },
	{ 0x66cebfe, "dma_unmap_page_attrs" },
	{ 0x6c847d77, "__pci_register_driver" },
	{ 0xbcd5e8f, "dma_sync_single_for_device" },
	{ 0x825e4adf, "vCanInit" },
	{ 0x59934c96, "pci_request_regions" },
	{ 0x69acdf38, "memcpy" },
	{ 0x37a0cba, "kfree" },
	{ 0x833811ec, "vCanGetCardInfo" },
	{ 0x206ebad6, "queue_irq_lock" },
	{ 0xa0eeba40, "pcpu_hot" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0xe2964344, "__wake_up" },
	{ 0xe5902880, "set_capability_mask" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x4185fd80, "pci_unregister_driver" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x6f1f8faf, "dev_driver_string" },
	{ 0xdf2ebb87, "_raw_read_unlock_irqrestore" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0x21a5d67f, "dma_map_page_attrs" },
	{ 0x6b2dc060, "dump_stack" },
	{ 0x122c3a7e, "_printk" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xbde0616d, "vCanGetCardInfo2" },
	{ 0xaa81651c, "set_capability_ex_value" },
	{ 0xa916b694, "strnlen" },
	{ 0xb1342cdb, "_raw_read_lock_irqsave" },
	{ 0xadc10e64, "vCanInitData" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x7cd8d75e, "page_offset_base" },
	{ 0x70866bac, "pci_clear_master" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x24d273d1, "add_timer" },
	{ 0x235ea4c1, "calculateCRC32" },
	{ 0x69dd3b5b, "crc32_le" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0x4b750f53, "_raw_spin_unlock_irq" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0xf48c0996, "set_capability_ex_mask" },
	{ 0xf6149df4, "set_capability_value" },
	{ 0x4c9d28b0, "phys_base" },
	{ 0x449ad0a7, "memcmp" },
	{ 0x9ed12e20, "kmalloc_large" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x96010d72, "pci_iounmap" },
	{ 0x265917c7, "vCanDispatchEvent" },
	{ 0xf93dd764, "dma_sync_single_for_cpu" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0xfb578fc5, "memset" },
	{ 0x7e26b97a, "pci_set_master" },
	{ 0x25974000, "wait_for_completion" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x679e43d1, "queue_empty" },
	{ 0xbfc177bc, "iowrite32_rep" },
	{ 0x1905693e, "vCanRemoveCardChannel" },
	{ 0xeb078aee, "_raw_write_unlock_irqrestore" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x97651e6c, "vmemmap_base" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0xc2941a76, "pci_release_regions" },
	{ 0x2cf56265, "__dynamic_pr_debug" },
	{ 0x56470118, "__warn_printk" },
	{ 0xaeda30e, "pci_disable_device" },
	{ 0x2bfda2ab, "dma_set_mask" },
	{ 0xb236c6eb, "kmalloc_trace" },
	{ 0x5021bd81, "_raw_write_lock_irqsave" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x6438f12, "vCanCleanup" },
	{ 0xf9a482f9, "msleep" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x1a103589, "kmalloc_caches" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x188ea314, "jiffies_to_timespec64" },
	{ 0x43581fb4, "module_layout" },
};

MODULE_INFO(depends, "kvcommon");


MODULE_INFO(srcversion, "E4BA0CAFC19829AA496EFC2");
