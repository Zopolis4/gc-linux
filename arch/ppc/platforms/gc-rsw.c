/* ------------------------------------------------------------------------- */
/* gc-rsw.c GameCube Reset Switch Driver                                     */
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

#define RSW_IRQ 1

static irqreturn_t gc_rsw_handler(int this_irq, void *dev_id, struct pt_regs *regs) {

	printk(KERN_ERR "gc_rsw: reset switch pressed\n");
	return IRQ_HANDLED;
}


static int gc_rsw_init(void)
{
	int err;

	err = request_irq(RSW_IRQ, gc_rsw_handler, 0, "GameCube Reset Switch", 0);
	if (err)
		printk(KERN_ERR "gc_rsw: Request irq%d failed\n", RSW_IRQ);

	return err;
}

static void gc_rsw_exit(void)
{
	free_irq(RSW_IRQ, 0);
}

MODULE_AUTHOR("Stefan Esser <se@nopiracy.de>");
MODULE_DESCRIPTION("GameCube Reset switch driver");
MODULE_LICENSE("GPL");

module_init(gc_rsw_init);
module_exit(gc_rsw_exit);
