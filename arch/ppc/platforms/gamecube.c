#include <linux/config.h>
#include <linux/init.h>
#include <linux/pagemap.h>
#include <linux/irq.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include "console.h"

void __init
gamecube_setup_arch(void)
{
}

unsigned long gamecube_find_end_of_memory(void)
{
	return 8*1024*1024;
}

void __init
gamecube_map_io(void)
{
	io_block_mapping(0xd0000000, 0, 0x02000000, _PAGE_IO);
}

static void __init
gamecube_init_IRQ(void)
{
}

int gamecube_get_irq(struct pt_regs* regs)
{
	return -1;
}

void __init gamecube_calibrate_decr(void)
{
	int freq, divisor;
	freq = 162000000 / 2;
	divisor = 4;
	tb_ticks_per_jiffy = freq / HZ / divisor;
	tb_to_us = mulhwu_scale_factor(freq/divisor, 1000000);
}

void __init
platform_init(unsigned long r3, unsigned long r4, unsigned long r5,
	      unsigned long r6, unsigned long r7)
{
	ppc_md.setup_arch = gamecube_setup_arch;
	ppc_md.setup_io_mappings = gamecube_map_io;
	ppc_md.find_end_of_memory = gamecube_find_end_of_memory;
	
	ppc_md.init_IRQ = gamecube_init_IRQ;
	ppc_md.get_irq = gamecube_get_irq;
	
	ppc_md.calibrate_decr = gamecube_calibrate_decr;
	
//	console_do_init();
}
