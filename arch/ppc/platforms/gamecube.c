#include <linux/config.h>
#include <linux/init.h>
#include <linux/pagemap.h>
#include <linux/irq.h>
#include <linux/console.h>
#include <linux/initrd.h>
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
	io_block_mapping(0xcc000000, 0x0c000000, 0x00100000, _PAGE_IO); /* GC IO */
}

static void
gamecube_unmask_irq(unsigned int irq)
{
	//printk("unmask(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM), GAMECUBE_IN(GAMECUBE_PIIC));
	if (irq < GAMECUBE_IRQS) {
		GAMECUBE_OUT(GAMECUBE_PIIM, GAMECUBE_IN(GAMECUBE_PIIM) | (1 << irq));
	}
	//printk("after unmask(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM), GAMECUBE_IN(GAMECUBE_PIIC));
}

static void
gamecube_mask_irq(unsigned int irq)
{
	//printk("mask(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM), GAMECUBE_IN(GAMECUBE_PIIC));
	if (irq < GAMECUBE_IRQS) {
		GAMECUBE_OUT(GAMECUBE_PIIM, GAMECUBE_IN(GAMECUBE_PIIM) & ~(1 << irq)); /* mask */
	}
}

static void
gamecube_mask_and_ack_irq(unsigned int irq)
{
	//printk("mask_and_ack(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM), GAMECUBE_IN(GAMECUBE_PIIC));
	if (irq < GAMECUBE_IRQS) {
		GAMECUBE_OUT(GAMECUBE_PIIM, GAMECUBE_IN(GAMECUBE_PIIM) & ~(1 << irq)); /* mask */
		GAMECUBE_OUT(GAMECUBE_PIIC, 1 << irq); /* ack */
	}
	//printk("after mask_and_ack(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM), GAMECUBE_IN(GAMECUBE_PIIC));
}

static struct hw_interrupt_type gamecube_pic = {
	"GameCube PIC",
	NULL, /* startup */
	NULL, /* shutdown */
	gamecube_unmask_irq,
	gamecube_mask_irq,
	gamecube_mask_and_ack_irq,
	NULL, /* end */
	NULL /* set_affinity */
};

static void __init
gamecube_init_IRQ(void)
{
	int i;

	GAMECUBE_OUT(GAMECUBE_PIIM,0);		/* disable all irqs */
	GAMECUBE_OUT(GAMECUBE_PIIC,0xffffffff);	/* ack all irqs */

	for (i = 0; i < GAMECUBE_IRQS; i++) {
		irq_desc[i].handler = &gamecube_pic;
	}
}

/*
 * Find the highest IRQ that generating an interrupt, if any.
 */
int
gamecube_get_irq(struct pt_regs *regs)
{
	int irq = 0;
	u_int irq_status, irq_test = 1;

	//printk("get_irq(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM), GAMECUBE_IN(GAMECUBE_PIIC));
	irq_status = GAMECUBE_IN(GAMECUBE_PIIC) & GAMECUBE_IN(GAMECUBE_PIIM);

	if(irq_status==0) {
		return -1;
		//printk("\nPanic: IRQ for no reason!\n\n\n\n\n\n");
		//while(1);
	}
	do
	{
		if (irq_status & irq_test)
			break;
		irq++;
		irq_test <<= 1;
	} while (irq < GAMECUBE_IRQS);

	return irq;
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
	freq = 162000000 / 2;
	divisor = 4;
	tb_ticks_per_jiffy = freq / HZ / divisor;
	tb_to_us = mulhwu_scale_factor(freq/divisor, 1000000);
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
	ppc_md.setup_io_mappings = gamecube_map_io;
	ppc_md.find_end_of_memory = gamecube_find_end_of_memory;

	ppc_md.init_IRQ = gamecube_init_IRQ;
	ppc_md.get_irq = gamecube_get_irq;

	ppc_md.restart = gamecube_restart;
	ppc_md.power_off = gamecube_power_off;
	ppc_md.halt = gamecube_halt;

	ppc_md.calibrate_decr = gamecube_calibrate_decr;

#ifdef CONFIG_DUMMY_CONSOLE
	conswitchp = &dummy_con;
#endif
}
