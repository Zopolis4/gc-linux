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

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include <asm/irq.h>
#include "gamecube.h"

#define DVD_IRQ 2

static irqreturn_t gc_dvdcover_handler(int this_irq, void *dev_id, struct pt_regs *regs) {

	unsigned long reason = GAMECUBE_IN(GAMECUBE_DICVR);

	// really a DVD cover interrupt?
	if (reason & 4) {
		GAMECUBE_OUT(GAMECUBE_DICVR, reason | 4);
		printk(KERN_ERR "gc_dvdcover: DVD cover was closed\n");
	}
	return IRQ_HANDLED;
}


static int gc_dvdcover_init(void)
{
	if (request_irq(DVD_IRQ, gc_dvdcover_handler, 0, "GameCube DVD Cover", 0) < 0) {
		printk(KERN_ERR "gc_dvdcover: Request irq%d failed\n", DVD_IRQ);
	} else {
		enable_irq(DVD_IRQ);
	}
	GAMECUBE_OUT(GAMECUBE_DICVR, GAMECUBE_IN(GAMECUBE_DICVR) | 2);
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
