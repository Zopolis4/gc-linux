/* ------------------------------------------------------------------------- */
/* gc-rsw.c GameCube Reset Switch Driver                                     */
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>

#ifdef CONFIG_KEXEC
#include <linux/kexec.h>
#endif

#define RSW_IRQ 1

/* from kernel/sys.c */
extern void ctrl_alt_del(void);

#define GC_RSW_NORMAL_TIMEOUT      3	/* secs */
#define GC_RSW_EMERGENCY_CLICKS   10

typedef enum {
	IDLE = 0,		/* nothing to do */
	NORMAL_RESET,		/* reboot requested */
	EMERGENCY_RESET,	/* try emergency reboot */
} gc_rsw_state_t;

struct gc_rsw_private {
	gc_rsw_state_t state;
	struct timer_list timer;
	unsigned long jiffies;
	int clicks;
	int timeout;
	spinlock_t lock;
};

static struct gc_rsw_private gc_rsw_private = {
	.state = IDLE,
	.timeout = GC_RSW_NORMAL_TIMEOUT,
};

static void gc_rsw_emergency_reset(void);
static void gc_rsw_normal_reset(unsigned long dummy);

/**
 *
 */
static irqreturn_t gc_rsw_handler(int this_irq, void *data,
				  struct pt_regs *regs)
{
	struct gc_rsw_private *priv = (struct gc_rsw_private *)data;
	unsigned long flags;

	spin_lock_irqsave(priv->lock, flags);

	/* someone pressed the reset button */
	switch (priv->state) {
	case IDLE:
		priv->state = NORMAL_RESET;
		printk(KERN_EMERG "Rebooting in %d seconds...\n",
		       priv->timeout);
		printk(KERN_WARNING
		       "Press Reset button again to cancel reboot!\n");

		/* schedule a reboot in a few seconds */
		init_timer(&priv->timer);
		priv->timer.expires = jiffies + priv->timeout * HZ;
		priv->timer.function =
		    (void (*)(unsigned long))gc_rsw_normal_reset;
		add_timer(&priv->timer);
		priv->jiffies = jiffies;
		break;
	case NORMAL_RESET:
		if (time_before(jiffies, priv->jiffies + priv->timeout * HZ)) {
			/* the reset button was hit again before deadline */
			del_timer(&priv->timer);
			priv->state = IDLE;
			printk(KERN_EMERG "Reboot cancelled!\n");
		} else {
			/*
			 * Time expired. System should be now restarting.
			 * Go to emergency mode in case something goes bad.
			 */
			priv->state = EMERGENCY_RESET;
			priv->clicks = 0;
			printk(KERN_WARNING
			       "SWITCHED TO EMERGENCY RESET MODE!\n"
			       "Press %d times the Reset button to force"
			       " a hard reset!\n", GC_RSW_EMERGENCY_CLICKS);
		}
		break;
	case EMERGENCY_RESET:
		/* force a hard reset if the user insists ... */
		if (++priv->clicks >= GC_RSW_EMERGENCY_CLICKS) {
			spin_unlock_irqrestore(priv->lock, flags);
			gc_rsw_emergency_reset();
			return IRQ_HANDLED;
		} else {
			printk(KERN_INFO
			       "%d ...\n",
			       GC_RSW_EMERGENCY_CLICKS - priv->clicks);
		}
		break;
	}

	spin_unlock_irqrestore(priv->lock, flags);

	return IRQ_HANDLED;
}

/**
 *
 */
static void gc_rsw_emergency_reset(void)
{
#ifdef CONFIG_KEXEC
	struct kimage *image;
	image = xchg(&kexec_image, 0);
	if (image) {
		machine_kexec(image);
	}
#endif
	machine_restart(NULL);
}

/**
 *
 */
static void gc_rsw_normal_reset(unsigned long dummy)
{
	ctrl_alt_del();
}

/**
 *
 */
static int gc_rsw_init(void)
{
	int err;

	spin_lock_init(&gc_rsw_private.lock);

	err =
	    request_irq(RSW_IRQ, gc_rsw_handler, 0, "GameCube Reset Switch",
			(void *)&gc_rsw_private);
	if (err)
		printk(KERN_ERR "gc_rsw: Request irq%d failed\n", RSW_IRQ);

	return err;
}

/**
 *
 */
static void gc_rsw_exit(void)
{
	free_irq(RSW_IRQ, &gc_rsw_private);
}

MODULE_AUTHOR("Stefan Esser <se@nopiracy.de>");
MODULE_DESCRIPTION("GameCube Reset switch driver");
MODULE_LICENSE("GPL");

module_init(gc_rsw_init);
module_exit(gc_rsw_exit);
