/*
 * drivers/exi/gcn_exi_lite.c
 *
 * Nintendo GameCube EXpansion Interface support, "lite" version.
 * Copyright (C) 2004 The GameCube Linux Team
 *
 * Partly, depends on existing work by tmbinc.
 * This code will be replaced by apgo's EXI framework, when available.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

/*
 * We guarantee that the following functions work even if the exi_lite_init
 * initialization function has not been called:
 *     exi_select, exi_deselect, exi_read, exi_write
 */

#define EXI_DEBUG

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include "gcn-exi-lite.h"


#ifdef EXI_DEBUG
#  define DPRINTK(fmt, args...) \
          printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif


#define exi_printk(level, format, arg...) \
	printk(level "exi: " format , ## arg)


#define EXI_IRQ		  4

#define EXI_MAX_CHANNELS  3	/* channels on the EXI bus */
#define EXI_MAX_EVENTS    3	/* types of events on the EXI bus */

#define EXI_READ	  0
#define EXI_WRITE	  1


#define EXI_CSR_BASE 0xcc006800	/* EXI Channel Status Register */
#define   EXI_CSR_EXT         (1<<12)
#define   EXI_CSR_EXTINT      (1<<11)
#define   EXI_CSR_EXTINTMASK  (1<<10)
#define   EXI_CSR_CSMASK      (0x7<<7)
#define     EXI_CSR_CS_0      (0x1<<7)	/* Chip Select 001 */
#define     EXI_CSR_CS_1      (0x2<<7)	/* Chip Select 010 */
#define     EXI_CSR_CS_2      (0x4<<7)	/* Chip Select 100 */
#define   EXI_CSR_CLKMASK     (0x7<<4)
#define     EXI_CSR_CLK_1MHZ  (0x0<<4)
#define     EXI_CSR_CLK_2MHZ  (0x1<<4)
#define     EXI_CSR_CLK_4MHZ  (0x2<<4)
#define     EXI_CSR_CLK_8MHZ  (0x3<<4)
#define     EXI_CSR_CLK_16MHZ (0x4<<4)
#define     EXI_CSR_CLK_32MHZ (0x5<<4)
#define   EXI_CSR_TCINT       (1<<3)
#define   EXI_CSR_TCINTMASK   (1<<2)
#define   EXI_CSR_EXIINT      (1<<1)
#define   EXI_CSR_EXIINTMASK  (1<<0)

#define EXI0_CSR (EXI_CSR_BASE + 0*0x14)	/* Channel 0 CSR */
#define EXI1_CSR (EXI_CSR_BASE + 1*0x14)	/* Channel 1 CSR */
#define EXI2_CSR (EXI_CSR_BASE + 2*0x14)	/* Channel 2 CSR */


struct exi_dev_event {
	int (*handler) (int, int, void *);
	void *data;
};

struct exi_private {
	spinlock_t lock;

	spinlock_t select_lock;
	unsigned long select_flags;

	struct exi_dev_event events[EXI_MAX_CHANNELS][EXI_MAX_EVENTS];
};


static struct exi_private __exi_private = {
	.lock = SPIN_LOCK_UNLOCKED,
	.select_lock = SPIN_LOCK_UNLOCKED,
};
#define exi_priv() (&__exi_private)




/* --------8<--------8<--------8<--------8<--------8<--------8<-------- */
/*
 * Here follows some glue to the external code.
 * --Albert Herranz
 */

/*
 * These functions should be static, but let's not touch a bit of
 * the external copy'n'pasted code...
 */
void __exi_select(int, int, int);
void __exi_deselect(int);
void __exi_imm(int, void *, int, int, int);
void __exi_sync(int);
void __exi_imm_ex(int, void *, int, int);

/* we won't let the external code alter our namespace too much */
#define exi_select __exi_select
#define exi_deselect __exi_deselect
#define exi_imm __exi_imm
#define exi_sync __exi_sync
#define exi_imm_ex __exi_imm_ex

/*
 * The included code comes directly from tmbinc's ipl replacement
 */
#include "gcn-exi-lite-tmbinc-exi_c.c"

/* final namespace cleanup */
#undef exi_select
#undef exi_deselect
#undef exi_imm
#undef exi_sync
#undef exi_imm_ex
/* --------8<--------8<--------8<--------8<--------8<--------8<-------- */




/**
 *
 */
static void inline exi_read_or_write(int channel, void *data, int len, int mode)
{
	if (__builtin_constant_p(len)) {
		switch(len) {
			case 1:
			case 2:
			case 3:
			case 4:
				__exi_imm(channel, data, len, mode, 0);
				__exi_sync(channel);
				break;
			default:
				__exi_imm_ex(channel, data, len, mode);
				break;
		}
	} else {
		__exi_imm_ex(channel, data, len, mode);
	}
}

/**
 *
 */
void exi_select(int channel, int device, int freq)
{
	struct exi_private *priv = exi_priv();

	spin_lock_irqsave(priv->select_lock, priv->select_flags);
	__exi_select(channel, device, freq);
}

/**
 *
 */
void exi_deselect(int channel)
{
	struct exi_private *priv = exi_priv();

	__exi_deselect(channel);
	spin_unlock_irqrestore(priv->select_lock, priv->select_flags);
}

/**
 *
 */
void exi_read(int channel, void *data, int len)
{
        exi_read_or_write(channel, data, len, EXI_READ);
}
                                                                                
/**
 *
 */
void exi_write(int channel, void *data, int len)
{
        exi_read_or_write(channel, data, len, EXI_WRITE);
}

/**
 *
 */
static inline int exi_trigger_event(int channel, int event_id)
{
	struct exi_private *priv = exi_priv();
	struct exi_dev_event *event;

	if (!priv)
		return -EINVAL;

	event = &priv->events[channel][event_id];
	if (event->handler) {
		return event->handler(channel, event_id, event->data);
	}
	return 0;
}

/**
 *
 */
int exi_register_event(int channel, int event_id,
		       int (*handler) (int, int, void *), void *data)
{
	struct exi_private *priv = exi_priv();
	struct exi_dev_event *event;

	if (channel < 0 || channel >= EXI_MAX_CHANNELS)
		return -EINVAL;
	if (event_id < 0 || event_id >= EXI_MAX_EVENTS)
		return -EINVAL;
	if (!priv)
		return -EINVAL;

	/* register event if free */
	event = &priv->events[channel][event_id];
	if (event->handler) {
		return -EBUSY;
	}
	event->handler = handler;
	event->data = data;

	/* ack and enable interrupts */
	unsigned long reg = EXI_CSR_BASE + channel * 0x14;
	unsigned long csr = readl(reg);
	switch (event_id) {
	case EXI_EVENT_INSERT:
		writel(csr | (EXI_CSR_EXTINT | EXI_CSR_EXTINTMASK), reg);
		break;
	case EXI_EVENT_TC:
		writel(csr | (EXI_CSR_TCINT | EXI_CSR_TCINTMASK), reg);
		break;
	case EXI_EVENT_IRQ:
		writel(csr | (EXI_CSR_EXIINT | EXI_CSR_EXIINTMASK), reg);
		break;
	}
	return 0;
}

/**
 *
 */
int exi_unregister_event(int channel, int event_id)
{
	struct exi_private *priv = exi_priv();
	struct exi_dev_event *event;

	if (channel < 0 || channel >= EXI_MAX_CHANNELS)
		return -EINVAL;
	if (event_id < 0 || event_id >= EXI_MAX_EVENTS)
		return -EINVAL;
	if (!priv)
		return -EINVAL;

	/* unregister event */
	event = &priv->events[channel][event_id];
	event->handler = 0;
	event->data = 0;

	/* ack and disable interrupts */
	unsigned long reg = EXI_CSR_BASE + channel * 0x14;
	unsigned long csr = readl(reg);
	switch (event_id) {
	case EXI_EVENT_INSERT:
		writel((csr | EXI_CSR_EXTINT) & ~EXI_CSR_EXTINTMASK, reg);
		break;
	case EXI_EVENT_TC:
		writel((csr | EXI_CSR_TCINT) & ~EXI_CSR_TCINTMASK, reg);
		break;
	case EXI_EVENT_IRQ:
		writel((csr | EXI_CSR_EXIINT) & ~EXI_CSR_EXIINTMASK, reg);
		break;
	}
	return 0;
}

/**
 *	exi_irq_handler - handle interrupts from EXI devices
 *	@irq: interrupt line
 *	@dev_id: device from where the interrupt comes
 *	@regs: processor register set
 *
 *	Handles External Interface interrupts from EXI devices.
 */
static irqreturn_t exi_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	struct exi_private *priv = exi_priv();

	register unsigned long reg, csr, status, mask;
	register int channel;

	spin_lock(&priv->lock);

	for (channel = 0; channel < EXI_MAX_CHANNELS; channel++) {
		reg = EXI_CSR_BASE + channel * 0x14;
		csr = readl(reg);
		mask = csr & (EXI_CSR_EXTINTMASK |
			      EXI_CSR_TCINTMASK | EXI_CSR_EXIINTMASK);
		status = csr & (mask << 1);
		if (!status)
			continue;

		writel(csr | status, reg);	/* ack all for this channel */

		if (status & EXI_CSR_EXTINT)
			exi_trigger_event(channel, EXI_EVENT_INSERT);
		if (status & EXI_CSR_TCINT)
			exi_trigger_event(channel, EXI_EVENT_TC);
		if (status & EXI_CSR_EXIINT)
			exi_trigger_event(channel, EXI_EVENT_IRQ);
	}

	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

/**
 *
 */
int exi_lite_init()
{
	struct exi_private *priv = exi_priv();
	int err = 0;

	spin_lock_init(priv->lock);
	spin_lock_init(priv->select_lock);

	err = request_irq(EXI_IRQ, exi_irq_handler, SA_SHIRQ, "exi", NULL);
	if (err) {
		exi_printk(KERN_ERR, "request of irq%d failed\n", EXI_IRQ);
	}

	return err;
}

/**
 *
 */
void exi_lite_exit(void)
{
	free_irq(EXI_IRQ, NULL);
}

