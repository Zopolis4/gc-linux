/*
 * arch/powerpc/platforms/embedded6xx/gamecube.c
 *
 * Nintendo GameCube board-specific support
 * Copyright (C) 2004-2008 The GameCube Linux Team
 * Copyright (C) 2007,2008 Albert Herranz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/initrd.h>
#include <linux/seq_file.h>
#include <linux/kexec.h>

#include <asm/io.h>
#include <asm/time.h>
#include <asm/bitops.h>
#include <asm/machdep.h>
#include <asm/pgtable.h>
#include <asm/prom.h>
#include <asm/lmb.h>

#include "gamecube.h"
#include "ugecon.h"

/*
 * These are used in setup_arch. *
 */
#define CSR_REG                 ((void __iomem *)(GCN_IO1_BASE+0x500A))
#define  DSP_CSR_PIINT          (1<<1)
#define  DSP_CSR_AIDINT         (1<<3)
#define  DSP_CSR_ARINT          (1<<5)
#define  DSP_CSR_DSPINT         (1<<7)

#define AUDIO_DMA_LENGTH        ((void __iomem *)(GCN_IO1_BASE+0x5036))
#define  AI_DCL_PLAY            (1<<15)



static void __init gamecube_progress(char *s, unsigned short hex)
{
#ifdef CONFIG_USBGECKO_EARLY_CONSOLE
	if (s)
		ug_early_puts(s);
	ug_early_puts("\n");
#endif
}

/*
 * FIXME
 * We have to get rid of these mappings and move to ioremap.
 */

extern int map_page(unsigned long va, phys_addr_t pa, int flags);

static void dirty_io_block_mapping(unsigned long va, phys_addr_t pa,
				   unsigned int size, int flags)
{
	int i;

	for (i = 0; i < size; i += PAGE_SIZE)
		map_page(va + i, pa + i, flags);
}

static void gamecube_setup_io_mappings(void)
{
#ifdef CONFIG_GAMECUBE_DEBUG_CONSOLE
	/* mapping for the debug console framebuffer */
	dirty_io_block_mapping(0xd0000000, 0, 0x02000000, _PAGE_IO);
#endif

	/* access to hardware registers */
#ifdef CONFIG_GAMECUBE_WII
	dirty_io_block_mapping(0xcd000000, 0x0d000000, 0x00010000, _PAGE_IO);
#endif
	dirty_io_block_mapping(0xcc000000, 0x0c000000, 0x00010000, _PAGE_IO);
}


static void gamecube_restart(char *cmd)
{
	local_irq_disable();
	out_8(FLIPPER_RESET, 0x00);
}

static void gamecube_power_off(void)
{
	local_irq_disable();
	for (;;); /* spin until power button pressed */
}

static void gamecube_halt(void)
{
	gamecube_restart(NULL);
}

static unsigned int gamecube_get_irq(void)
{
	int irq;
	u32 irq_status;

	irq_status = in_be32(FLIPPER_ICR) & in_be32(FLIPPER_IMR);
	if (irq_status == 0)
		return -1;	/* no more IRQs pending */

        __asm __volatile ("cntlzw %0,%1": "=r"(irq) : "r"(irq_status));

	return (31 - irq);
}

static void flipper_mask_and_ack_irq(unsigned int irq)
{
	clear_bit(irq, FLIPPER_IMR);
	set_bit(irq, FLIPPER_ICR);
}

static void flipper_mask_irq(unsigned int irq)
{
	clear_bit(irq, FLIPPER_IMR);
}

static void flipper_unmask_irq(unsigned int irq)
{
	set_bit(irq, FLIPPER_IMR);
}

static void flipper_end_irq(unsigned int irq)
{
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS))
	    && irq_desc[irq].action)
		flipper_unmask_irq(irq);
}

static struct hw_interrupt_type flipper_pic = {
	.typename	= "flipper-pic",
	.enable		= flipper_unmask_irq,
	.disable	= flipper_mask_irq,
	.ack		= flipper_mask_and_ack_irq,
	.end		= flipper_end_irq,
};

static void gamecube_init_irq(void)
{
	int i;

	/* mask and ack all IRQs */
	out_be32(FLIPPER_IMR, 0x00000000);
	out_be32(FLIPPER_ICR, 0xffffffff);

	for (i = 0; i < FLIPPER_NR_IRQS; i++)
		irq_desc[i].chip = &flipper_pic;

	ppc_md.get_irq = gamecube_get_irq;
}

static void gamecube_show_cpuinfo(struct seq_file *m)
{
	seq_printf(m, "vendor\t\t: IBM\n");
	seq_printf(m, "machine\t\t: Nintendo GameCube\n");
}

static void gamecube_setup_arch(void)
{
#ifdef CONFIG_GAMECUBE_DEBUG_CONSOLE
	gcn_con_init();
#endif
#ifdef CONFIG_USBGECKO_EARLY_CONSOLE
	ug_early_con_init();
#endif

#if 0
        /* ack and clear the interrupts for the AI line */
        out_be16(CSR_REG,
                 DSP_CSR_PIINT|DSP_CSR_AIDINT|DSP_CSR_ARINT|DSP_CSR_DSPINT);
        /* stop any audio */
        out_be16(AUDIO_DMA_LENGTH,
                 in_be16(AUDIO_DMA_LENGTH) & ~AI_DCL_PLAY);
#endif
}

static int __init gamecube_probe(void)
{
        unsigned long dt_root;

        dt_root = of_get_flat_dt_root();
        if (!of_flat_dt_is_compatible(dt_root, "nintendo,gamecube"))
                return 0;

        return 1;
}

#ifdef CONFIG_KEXEC
static void gamecube_shutdown(void)
{
	/* currently not used */
}

static int gamecube_kexec_prepare(struct kimage *image)
{
	return 0;
}
#endif /* CONFIG_KEXEC */



define_machine(gamecube) {
	.name			= "gamecube",
	.probe			= gamecube_probe,
	.setup_arch		= gamecube_setup_arch,
	.setup_io_mappings	= gamecube_setup_io_mappings,
	.show_cpuinfo		= gamecube_show_cpuinfo,
	.init_IRQ		= gamecube_init_irq,
	.calibrate_decr		= generic_calibrate_decr,
	.restart		= gamecube_restart,
	.power_off		= gamecube_power_off,
	.halt			= gamecube_halt,
	.progress		= gamecube_progress,
#ifdef CONFIG_KEXEC
	.machine_shutdown	= gamecube_shutdown,
	.machine_kexec_prepare	= gamecube_kexec_prepare,
	.machine_kexec		= default_machine_kexec,
#endif
};

