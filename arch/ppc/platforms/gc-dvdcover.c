/* ------------------------------------------------------------------------- */
/* gc-dvdcover.c GameCube DVD Cover Close Message Driver                     */
/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2004 Stefan Esser
     Copyright (C) 2004 Albert Herranz

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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include <asm/io.h>

#define DVD_IRQ			2

#define DI_DISR              0xcc006000	/* DI Status Register */
#define  DI_DISR_BRKINT      (1<<6)
#define  DI_DISR_BRKINTMASK  (1<<5)
#define  DI_DISR_TCINT       (1<<4)
#define  DI_DISR_TCINTMASK   (1<<3)
#define  DI_DISR_DEINT       (1<<2)
#define  DI_DISR_DEINTMASK   (1<<1)
#define  DI_DISR_BRK         (1<<0)

#define DI_DICVR             0xcc006004	/* DI Cover Register */
#define  DI_DICVR_CVRINT     (1<<2)
#define  DI_DICVR_CVRINTMASK (1<<1)
#define  DI_DICVR_CVR        (1<<0)

#define DI_DICMDBUF0         0xcc006008	/* DI Command Buffer 0 */

#define DI_DICR              0xcc00601c	/* DI Control Register */
#define  DI_DICR_RW          (1<<2)
#define  DI_DICR_DMA         (1<<1)
#define  DI_DICR_TSTART      (1<<0)

#define DI_CMD_STOP          (0xE3)

/**
 *
 */
static irqreturn_t gc_dvdcover_handler(int this_irq, void *dev_id,
				       struct pt_regs *regs)
{
	unsigned long reason = readl(DI_DICVR);

	/* handle only DVD cover interrupts here */
	if (reason & DI_DICVR_CVRINT) {
		writel(reason | DI_DICVR_CVRINT, DI_DICVR);
		printk(KERN_INFO "gc_dvdcover: DVD cover was %s.\n",
		       (reason & DI_DICVR_CVR) ? "opened" : "closed");

		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

/**
 *
 */
static int gc_dvdcover_init(void)
{
	unsigned long outval;
	int err;

	/* clear pending DI interrupts and mask new ones */
	/* this prevents an annoying bug while we lack a complete DVD driver */
	outval = DI_DISR_BRKINT | DI_DISR_TCINT | DI_DISR_DEINT;
	outval &= ~(DI_DISR_BRKINTMASK | DI_DISR_TCINTMASK | DI_DISR_DEINTMASK);
	writel(outval, DI_DISR);

	/* stop DVD motor */
	writel(DI_CMD_STOP << 24, DI_DICMDBUF0);
	writel(DI_DICR_TSTART, DI_DICR);

	err =
	    request_irq(DVD_IRQ, gc_dvdcover_handler, 0, "GameCube DVD Cover",
			0);
	if (err)
		return err;

	writel(readl(DI_DICVR) | DI_DICVR_CVRINTMASK, DI_DICVR);

	return 0;
}

/**
 *
 */
static void gc_dvdcover_exit(void)
{
	free_irq(DVD_IRQ, 0);
}

MODULE_AUTHOR("Stefan Esser <se@nopiracy.de>");
MODULE_DESCRIPTION("GameCube DVD cover close message driver");
MODULE_LICENSE("GPL");

module_init(gc_dvdcover_init);
module_exit(gc_dvdcover_exit);

