/* ------------------------------------------------------------------------- */
/* gc-dvdcover.c GameCube DVD Cover Close Message Driver                                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2004 Stefan Esser

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.		     */
/* ------------------------------------------------------------------------- */
/*
 * Albert Herranz
 *	GAMECUBE
 *	- Fixed bug that caused kernel to "loop" in gc_dvdcover_handler
 *	  sometimes during kernel init.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/irq.h>

#define DVD_IRQ			2

#define GAMECUBE_DICVR          0xcc006004 /* DI Cover Register */
#define GC_DI_DISR              0xcc006000 /* DI Status Register */
#define GC_DI_DISR_BRKINT       (1<<6)
#define GC_DI_DISR_BRKINTMASK   (1<<5)
#define GC_DI_DISR_TCINT        (1<<4)
#define GC_DI_DISR_TCINTMASK    (1<<3)
#define GC_DI_DISR_DEINT        (1<<2)
#define GC_DI_DISR_DEINTMASK    (1<<1)
#define GC_DI_DISR_BRK          (1<<0)


static irqreturn_t gc_dvdcover_handler(int this_irq, void *dev_id, struct pt_regs *regs) {

	unsigned long reason = readl(GAMECUBE_DICVR);

	// really a DVD cover interrupt?
	if (reason & 4) {
		writel(reason | 4, GAMECUBE_DICVR);
		printk(KERN_INFO "gc_dvdcover: DVD cover was closed\n");
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static int gc_dvdcover_init(void)
{
        unsigned long outval;
                                                                                
        /* clear pending DI interrupts and mask new ones */
        /* this prevents an annoying bug while we lack a complete DVD driver */
        outval = GC_DI_DISR_BRKINT | GC_DI_DISR_TCINT |
                        GC_DI_DISR_DEINT;
        outval &= ~(GC_DI_DISR_BRKINTMASK | GC_DI_DISR_TCINTMASK |
                        GC_DI_DISR_DEINTMASK);
        writel(outval, GC_DI_DISR);

	if (request_irq(DVD_IRQ, gc_dvdcover_handler, 0, "GameCube DVD Cover", 0) < 0) {
		printk(KERN_ERR "gc_dvdcover: Request irq%d failed\n", DVD_IRQ);
	} else {
		enable_irq(DVD_IRQ);
		writel(readl(GAMECUBE_DICVR) | 2, GAMECUBE_DICVR);
	}
	return 0;
}

static void gc_dvdcover_exit(void)
{
	disable_irq(DVD_IRQ);
	free_irq(DVD_IRQ, 0);
}

MODULE_AUTHOR("Stefan Esser <se@nopiracy.de>");
MODULE_DESCRIPTION("GameCube DVD cover close message driver");
MODULE_LICENSE("GPL");

module_init(gc_dvdcover_init);
module_exit(gc_dvdcover_exit);
