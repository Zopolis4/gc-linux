/*
 * arch/ppc/platforms/gamecube.c
 */

#include <linux/init.h>
#include <linux/config.h>
#include <linux/console.h>
#include <linux/initrd.h>
#include <linux/seq_file.h>

#include <asm/machdep.h>
#include <asm/bootinfo.h>
#include <asm/time.h>
#include <asm/io.h>

#include "console.h"
#include "gamecube.h"


extern void gamecube_init_IRQ(void);
extern int gamecube_get_irq(struct pt_regs *regs);

extern long __init
gamecube_time_init(void);

extern unsigned long 
gamecube_get_rtc_time(void);

extern int 
gamecube_set_rtc_time(unsigned long nowtime);

void __init
gamecube_setup_arch(void)
{
}

unsigned long gamecube_find_end_of_memory(void)
{
	return 24*1024*1024 - (640*576*2); /* 24 MB minus max. framebuffer */
}

void __init
gamecube_map_io(void)
{
	io_block_mapping(0xd0000000, 0, 0x02000000, _PAGE_IO);
	io_block_mapping(0xcc000000, 0x0c000000, 0x00100000, _PAGE_IO); /* GC IO */
}

static void
gamecube_restart(char *cmd)
{
	printk("gamecube_restart()\n");
	GAMECUBE_OUT(GAMECUBE_RESET, 0);
}

static void
gamecube_power_off(void)
{
	for(;;);
}

static void
gamecube_halt(void)
{
	gamecube_restart(NULL);
}

void __init gamecube_calibrate_decr(void)
{
	int freq, divisor;
	freq = 162000000;
	divisor = 4;
	tb_ticks_per_jiffy = freq / HZ / divisor;
	tb_to_us = mulhwu_scale_factor(freq/divisor, 1000000);
}

static int
gamecube_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "vendor\t\t: IBM\n");
	seq_printf(m, "machine\t\t: Nintendo GameCube\n");

	return 0;
}

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	parse_bootinfo(find_bootinfo());

#ifdef CONFIG_BLK_DEV_INITRD
	if ( r4 )
	{
		initrd_start = r4 + KERNELBASE;
		initrd_end = r5 + KERNELBASE;
	}
#endif

	ppc_md.setup_arch = gamecube_setup_arch;
	ppc_md.show_cpuinfo = gamecube_show_cpuinfo;

	ppc_md.init_IRQ = gamecube_init_IRQ;
	ppc_md.get_irq = gamecube_get_irq;

	ppc_md.restart = gamecube_restart;
	ppc_md.power_off = gamecube_power_off;
	ppc_md.halt = gamecube_halt;

	ppc_md.calibrate_decr = gamecube_calibrate_decr;

	ppc_md.find_end_of_memory = gamecube_find_end_of_memory;
	ppc_md.setup_io_mappings = gamecube_map_io;

	ppc_md.time_init      = gamecube_time_init;
	ppc_md.set_rtc_time   = gamecube_set_rtc_time;
	ppc_md.get_rtc_time   = gamecube_get_rtc_time;
#ifdef CONFIG_DUMMY_CONSOLE
	conswitchp = &dummy_con;
#endif
}
