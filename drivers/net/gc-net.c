/* ------------------------------------------------------------------------- */
/* gc-net.c GameCube BroadBand Adaptor Driver                                */
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <asm/system.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include <asm/io.h>

#include "exi.h"

#define BBA_IRQ 4
#define IRQ_EXI 4

void gcif_irq_handler(int channel, int event, void *ct);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// must be MOVED LATER
///////////////////////////////////////////////////////////////////////////////////////////////////////

/* exi_select: enable chip select, set speed */
void exi_select(int channel, int device, int freq)
{
	volatile unsigned long *exi = (volatile unsigned long *)0xCC006800;
	long d;
	// exi_select
	d = exi[channel * 5];
	d &= 0x405;
	d |= ((1<<device)<<7) | (freq << 4);
	exi[channel*5] = d;
}

/* disable chipselect */
void exi_deselect(int channel)
{
	volatile unsigned long *exi = (volatile unsigned long *)0xCC006800;
	exi[channel * 5] &= 0x405;
}

/* dirty way for asynchronous reads */
static void *exi_last_addr;
static int   exi_last_len;

/* mode?Read:Write len bytes to/from channel */
/* when read, data will be written back in exi_sync */
void exi_imm(int channel, void *data, int len, int mode, int zero)
{
	volatile unsigned long *exi = (volatile unsigned long *)0xCC006800;
	if (mode == EXI_WRITE)
		exi[channel * 5 + 4] = *(unsigned long*)data;
	exi[channel * 5 + 3] = ((len-1)<<4)|(mode<<2)|1;
	if (mode == EXI_READ)
	{
		exi_last_addr = data;
		exi_last_len = len;
	} else
	{
		exi_last_addr = 0;
		exi_last_len = 0;
	}
}

/* Wait until transfer is done, write back data */
void exi_sync(int channel)
{
	volatile unsigned long *exi = (volatile unsigned long *)0xCC006800;
	while (exi[channel * 5 + 3] & 1);

	if (exi_last_addr)
	{	
		int i;
		unsigned long d;
		d = exi[channel * 5 + 4];
		for (i=0; i<exi_last_len; ++i)
			((unsigned char*)exi_last_addr)[i] = (d >> ((3-i)*8)) & 0xFF;
	}
}

/* simple wrapper for transfers > 4bytes */
void exi_imm_ex(int channel, void *data, int len, int mode)
{
	unsigned char *d = (unsigned char*)data;
	while (len)
	{
		int tc = len;
		if (tc > 4)
			tc = 4;
		exi_imm(channel, d, tc, mode, 0);
		exi_sync(channel);
		len-=tc;
		d+=tc;
	}
}

static exi_irq_handler_t *exi_handler[3 * 3]; // 3 channels, 3 events
static void *exi_handler_context[3 * 3];
static unsigned long exi_enable_mask[3];

void exi_refresh_enable(void)
{
	int channel;
	for (channel = 0; channel < 3; ++channel)
		*(unsigned long*)(0xcc006800 + channel*0x14) |= 0x405; // exi_enable_mask[channel];
}

static inline int have_irq(int t)
{
	if (exi_handler[t])
	{
		printk("CALL HANDLER\n");
		exi_handler[t](t/3, t%3, exi_handler_context[t]);
		return 1;
	} else
	{
		printk("UNHANDLED EXI\n");
		return 0;
	}
}

void exi_interrupt_handler(int irq, void *c)
{
	int ch;
	for (ch = 0; ch < 3; ++ch)
	{
		unsigned long v = (*(unsigned long*)(0xcc006800 + ch * 0x14)); //  & 0xC0F;
		v &= v<<1;
		if (v & 0x800)
			have_irq(ch * 3 + EXI_EVENT_INSERT);
		if (v & 8)
			have_irq(ch * 3 + EXI_EVENT_TC);
		if (v & 2)
			have_irq(ch * 3 + EXI_EVENT_IRQ);
		*(unsigned long*)(0xcc006800 + ch * 0x14) |= v;
	}
}

void exi_interrupt_debug_handler(int irq, void *c)
{
	int ch;
	for (ch = 0; ch < 3; ++ch)
	{
		unsigned long v = (*(unsigned long*)(0xcc006800 + ch * 0x14)); //  & 0xC0F;
		v &= v<<1;
		printk("ch %d c %08x\n", ch, (unsigned int)v);
		if (v & 0x800)
			if (!have_irq(ch * 3 + EXI_EVENT_INSERT))
				; // v &= ~0x800;
		if (v & 8)
			if (!have_irq(ch * 3 + EXI_EVENT_TC))
				; // v &= ~8;
		if (v & 2)
			if (!have_irq(ch * 3 + EXI_EVENT_IRQ))
				; /// v &= ~2;
		*(unsigned long*)(0xcc006800 + ch * 0x14) |= v;
	}
}


void exi_init()
{
	printk("EXI: init interrupts\n");
	//request_irq(IRQ_EXI, exi_interrupt_handler, 0);
	//request_debug_irq(IRQ_EXI, exi_interrupt_debug_handler, 0);
	
	*(unsigned long*)(0xcc006814) |= 3<<10; // enable&clear irq for insertion
}

int exi_request_irq(int channel, int event, exi_irq_handler_t *handler, void *context)
{
	if (exi_handler[channel * 3 + event])
	{
		printk("EXI: irq %d:%d already used!\n", channel, event);
		return -1;
	}
	exi_handler[channel * 3 + event] = handler;
	exi_handler_context[channel * 3 + event] = context;
	
	switch (event)
	{
	case EXI_EVENT_TC:
		*(unsigned long*)(0xcc006800 + channel*0x14) |= 3<<2;  // enable&clear irq
		exi_enable_mask[channel] |= 1 << 2;
		break;
	case EXI_EVENT_INSERT:
		*(unsigned long*)(0xcc006800 + channel*0x14) |= 3<<10;
		exi_enable_mask[channel] |= 1 << 10;
		break;
	case EXI_EVENT_IRQ:
		*(unsigned long*)(0xcc006800 + channel*0x14) |= 3<<0;
		exi_enable_mask[channel] |= 1 << 0;
		break;
	}
	return 0;
}

int exi_free_irq(int channel, int event)
{
	exi_handler[channel * 3 + event] = 0;
	switch (event)
	{
	case EXI_EVENT_TC:
		*(unsigned long*)(0xcc006800 + channel*0x14) &= ~(3<<2);  // enable&clear irq
		exi_enable_mask[channel] &= ~(1 << 2);
		break;
	case EXI_EVENT_INSERT:
		*(unsigned long*)(0xcc006800 + channel*0x14) &= ~(3<<10);
		exi_enable_mask[channel] &= ~(1 << 10);
		break;
	case EXI_EVENT_IRQ:
		*(unsigned long*)(0xcc006800 + channel*0x14) &= ~(3<<0);
		exi_enable_mask[channel] &= ~(1 << 0);
		break;
	}
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////


unsigned char eth_inb(int reg)
{
	unsigned long val;
	unsigned char res;
	val = (reg << 8) | 0x80000000;
	exi_select(0, 2, 5);
	exi_imm(0, &val, 4, EXI_WRITE, 0);
	exi_sync(0);
	exi_imm(0, &res, 1, EXI_READ, 0);
	exi_sync(0);
	exi_deselect(0);
	return res;
}

unsigned char eth_exi_inb_slow(int reg)
{
	unsigned short val;
	unsigned char res;
	val = (reg << 8);
	exi_select(0, 2, 5);
	exi_imm(0, &val, 2, EXI_WRITE, 0);
	exi_sync(0);
	exi_imm(0, &res, 1, EXI_READ, 0);
	exi_sync(0);
	udelay(200);
	exi_deselect(0);
	return res;
}

void eth_outb(int reg, unsigned char byte)
{
	unsigned long val;
	val = (reg << 8)|0xC0000000;
	exi_select(0, 2, 5);
	exi_imm(0, &val, 4, EXI_WRITE, 0);
	exi_sync(0);
	exi_imm(0, &byte, 1, EXI_WRITE, 0);
	exi_sync(0);
	exi_deselect(0);
}

void eth_exi_outb(int reg, unsigned char byte)
{
	unsigned short val;
	val = (reg << 8)|0x4000;
	exi_select(0, 2, 5);
	exi_imm(0, &val, 2, EXI_WRITE, 0);
	exi_sync(0);
	exi_imm(0, &byte, 1, EXI_WRITE, 0);
	exi_sync(0);
	exi_deselect(0);
}

unsigned char eth_exi_inb(int reg)
{
	unsigned short val;
	unsigned char res;
	val = reg << 8;
	exi_select(0, 2, 5);
	exi_imm(0, &val, 2, EXI_WRITE, 0);
	exi_sync(0);
	exi_imm(0, &res, 1, EXI_READ, 0);
	exi_sync(0);
	exi_deselect(0);
	return res;
}

void eth_exi_ins(int reg, void *res, int len)
{
	unsigned short val;
	val = reg << 8;
	exi_select(0, 2, 5);
	exi_imm(0, &val, 2, EXI_WRITE, 0);
	exi_sync(0);
	exi_imm_ex(0, res, len, EXI_READ);
	exi_sync(0);
	exi_deselect(0);
}

void eth_exi_outs(int reg, void *res, int len)
{
	unsigned short val;
	val = (reg << 8)|0x4000;
	exi_select(0, 2, 5);
	exi_imm(0, &val, 2, EXI_WRITE, 0);
	exi_sync(0);
	exi_imm_ex(0, res, len, EXI_WRITE);
	exi_sync(0);
	exi_deselect(0);
}

void eth_ins(int reg, void *res, int len)
{
	unsigned long val;
	val = (reg << 8)|0x80000000;
	exi_select(0, 2, 5);
	exi_imm(0, &val, 4, EXI_WRITE, 0);
	exi_sync(0);
	exi_imm_ex(0, res, len, EXI_READ);
	exi_sync(0);
	exi_deselect(0);
}

void eth_outs(int reg, void *res, int len)
{
	unsigned long val;
	val = (reg << 8)|0xC0000000;
	exi_select(0, 2, 5);
	exi_imm(0, &val, 4, EXI_WRITE, 0);
	exi_sync(0);
	exi_imm_ex(0, res, len, EXI_WRITE);
	exi_sync(0);
	exi_deselect(0);
}


static spinlock_t		gc_bba_lock;
#define RUNT 60		/* Too small Ethernet packet */
#define ETH_LEN 6

/*
 * D-Link driver variables:
 */

static volatile int		rx_page;

#define TX_PAGES 2
static volatile int		tx_fifo[TX_PAGES];
static volatile int		tx_fifo_in;
static volatile int		tx_fifo_out;
static volatile int		free_tx_pages = TX_PAGES;
static int			was_down;


static irqreturn_t gc_bba_interrupt(int irq, void *dev_id, struct pt_regs * regs);
static int	adapter_init(struct net_device *dev);


/*

static inline u8 de600_read_status(struct net_device *dev)
{
	u8 status;

	outb_p(STATUS, DATA_PORT);
	status = inb(STATUS_PORT);
	outb_p(NULL_COMMAND | HI_NIBBLE, DATA_PORT);

	return status;
}

static inline u8 de600_read_byte(unsigned char type, struct net_device *dev)
{
	// dev used by macros 
	u8 lo;
	outb_p((type), DATA_PORT);
	lo = ((unsigned char)inb(STATUS_PORT)) >> 4;
	outb_p((type) | HI_NIBBLE, DATA_PORT);
	return ((unsigned char)inb(STATUS_PORT) & (unsigned char)0xf0) | lo;
}
*/

/*
 * Open/initialize the board.  This is called (in the current kernel)
 * after booting when 'ifconfig <dev->name> $IP_ADDR' is run (in rc.inet1).
 *
 * This routine should set everything up anew at each open, even
 * registers that "should" only need to be set once at boot, so that
 * there is a non-reboot way to recover if something goes wrong.
 */

static int gc_bba_open(struct net_device *dev)
{
	unsigned long flags;
	printk("gc_bba_open\n");
	
	int ret = request_irq(BBA_IRQ, gc_bba_interrupt, 0, dev->name, dev);
	if (ret) {
		printk(KERN_ERR "%s: unable to get IRQ %d\n", dev->name, BBA_IRQ);
		return ret;
	}
	spin_lock_irqsave(&gc_bba_lock, flags);
	ret = adapter_init(dev);
	spin_unlock_irqrestore(&gc_bba_lock, flags);
	return ret;
}

/*
 * The inverse routine to de600_open().
 */


static int gc_bba_close(struct net_device *dev)
{
	printk("gc_bba_close\n");

//	//select_nic();
	rx_page = 0;
//	de600_put_command(RESET);
//	de600_put_command(STOP_RESET);
//	de600_put_command(0);
//	//select_prn();
	free_irq(BBA_IRQ, dev);
	return 0;
}


static struct net_device_stats *get_stats(struct net_device *dev)
{
	return (struct net_device_stats *)(dev->priv);
}

static inline void trigger_interrupt(struct net_device *dev)
{
//	de600_put_command(FLIP_IRQ);
//	//select_prn();
//	DE600_SLOW_DOWN;
//	//select_nic();
//	de600_put_command(0);
}

/*
 * Copy a buffer to the adapter transmit page memory.
 * Start sending.
 */
 
static int gc_bba_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	unsigned long flags;
	int	transmit_from;
	int	len;
	int	tickssofar;
	u8	*buffer = skb->data;
	int	i;

	if (free_tx_pages <= 0) {	/* Do timeouts, to avoid hangs. */
		tickssofar = jiffies - dev->trans_start;
		if (tickssofar < 5)
			return 1;
		/* else */
		printk(KERN_WARNING "%s: transmit timed out (%d), %s?\n", dev->name, tickssofar, "network cable problem");
		/* Restart the adapter. */
		spin_lock_irqsave(&gc_bba_lock, flags);
		if (adapter_init(dev)) {
			spin_unlock_irqrestore(&gc_bba_lock, flags);
			return 1;
		}
		spin_unlock_irqrestore(&gc_bba_lock, flags);
	}

	/* Start real output */
	printk("gc_bba_start_xmit:len=%d, page %d/%d\n", skb->len, tx_fifo_in, free_tx_pages);

	if ((len = skb->len) < RUNT)
		len = RUNT;

	spin_lock_irqsave(&gc_bba_lock, flags);

	unsigned int val=0xC0004800;	// register 0x48 is the output queue
	
	struct pbuf *q;
	
	exi_select(0, 2, 5);
	exi_imm(0, &val, 4, EXI_WRITE, 0);
	exi_sync(0);
	
	exi_imm_ex(0, buffer, skb->len, EXI_WRITE);

	exi_deselect(0);

	val = eth_inb(0);
	if (val & 4)
	{
		printk("err USE!\n");
	spin_unlock_irqrestore(&gc_bba_lock, flags);
	dev_kfree_skb(skb);
		return 1;
	}
	val|=4;
	eth_outb(0, val);
	
//	printk("DEBUG: waiting for tx done!\n");

	while (!(eth_inb(9)&0x14));
	
	if (eth_inb(9) & 0x10)
	{
		printk("TRANSMIT ERROR!\n");
		eth_outb(9, 0x10);
	}
	
	if (eth_inb(9) & 0x4)
		eth_outb(9, 0x4);
	
	int s = eth_inb(4);
	if (s)
		printk("tx error %02x\n", s);
	

	spin_unlock_irqrestore(&gc_bba_lock, flags);
	dev_kfree_skb(skb);
	return 0;
}


static char *gc_input(struct net_device *dev)
{
//	struct gcif *gcif=(struct gcif*)netif->state;
	struct sk_buff	*skb;
	char *p, *q;
	unsigned char *buffer;
	unsigned short len, p_read, p_write;
	int i;
	int ptr;

	p_write  = eth_inb(0x16);
	p_write |= eth_inb(0x17) << 8;

	p_read  = eth_inb(0x18);
	p_read |= eth_inb(0x19) << 8;
//	printk("w %x r %x\n", p_write, p_read);
	
	if (p_read == p_write)
	{
		printk("nothing left.\n");
		return 0;
	}

	unsigned char descr[4];
	eth_outb(0x3a, 2);
	if (eth_inb(0x3a) & 2)
	{
		printk("NO DATA AVAILABLE!\n");
		return 0;
	}
	
	len=0;
	
	descr[0] = eth_inb((p_read<<8) + 0);
	descr[1] = eth_inb((p_read<<8) + 1);
	descr[2] = eth_inb((p_read<<8) + 2);
	descr[3] = eth_inb((p_read<<8) + 3);
	
	len = (descr[1] >> 4) | (descr[2]<<4);
	
	len-=4; //???

	if ((len < 32)  ||  (len > 1535)) {
		printk(KERN_WARNING "%s: Bogus packet size %d.\n", dev->name, len);
		if (len > 10000)
			adapter_init(dev);
		return;
	}

	skb = dev_alloc_skb(len+2);
	if (skb == NULL) {
		printk("%s: Couldn't allocate a sk_buff of size %d.\n", dev->name, len);
		return;
	}
	/* else */

	skb->dev = dev;
	skb_reserve(skb,2);	/* Align */

	/* 'skb->data' points to the start of sk_buff data area. */
	buffer = skb_put(skb,len);
	
	printk("There are %u bytes data\n", len);
	
	ptr = (p_read << 8) + 4;
	for (i=0; i<len; ++i)
	{
		if (ptr == 0x1000) // wrap around
			ptr = 0x100;
		buffer[i]=eth_inb(ptr++); // skip descriptor
	}
	
	eth_outb(0x18, descr[0] & 0xFF);
	eth_outb(0x19, descr[1] & 0xFF);

	skb->protocol=eth_type_trans(skb,dev);

	netif_rx(skb);

	/* update stats */
	dev->last_rx = jiffies;
	((struct net_device_stats *)(dev->priv))->rx_packets++; /* count all receives */
	((struct net_device_stats *)(dev->priv))->rx_bytes += len; /* count all received bytes */

	
//	i=0;
//	p = pbuf_alloc(PBUF_LINK, len, PBUF_POOL);
//	if(p != NULL) {
		/* We iterate over the pbuf chain until we have read the entire
			 packet into the pbuf. */
//		for(q = p; q != NULL; q = q->next) {
			/* Read enough bytes to fill this pbuf in the chain. The
				 avaliable data in the pbuf is given by the q->len
				 variable. */
			/* read data into(q->payload, q->len); */
//			memcpy(q->payload, buffer+i, q->len);
//			i+=q->len;
			// exi_imm_ex(0, q->payload, q->len, EXI_READ);
//		}
//	}

	return p;	
}


static void inline gcif_service(struct net_device *dev)
{
//	struct gcif *gcif;
//	gcif = netif->state;
	
	unsigned short  p_read, p_write;

	
	int status = eth_inb(9) & eth_inb(8);
	
	printk("gcif_service: status %08x\n", status);
	
	if (!status)
		printk("?? GC irq but no irq ??\n");
	
	if (status & 2)
	{
		while (1)
		{
			p_write  = eth_inb(0x16);
			p_write |= eth_inb(0x17) << 8;

			p_read  = eth_inb(0x18);
			p_read |= eth_inb(0x19) << 8;

			if (p_write == p_read)
				break;
			gc_input(dev);
printk("Break\n");
			break;
		}
		eth_outb(9, 2);
	}
	if (status & 8)
	{
		printk("receive error :(\n");
		eth_outb(9, 8);
	}
	if (status & ~(8|2))
	{
		printk("status %02x\n", status);
		eth_outb(9, ~(8|2));
	}
}


/*
 * The typical workload of the driver:
 * Handle the network interface interrupts.
 */

static irqreturn_t gc_bba_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct net_device	*dev = dev_id;
	u8		irq_status;
	int		retrig = 0;
	int		boguscount = 0;

	/* This might just as well be deleted now, no crummy drivers present :-) */
	if ((dev == NULL) || (BBA_IRQ != irq)) {
		printk(KERN_ERR "%s: bogus interrupt %d\n", dev?dev->name:"GC_BBA", irq);
		return IRQ_NONE;
	}

	spin_lock(&gc_bba_lock);


	int ch;
	for (ch = 0; ch < 3; ++ch)
	{
		unsigned long v = (*(unsigned long*)(0xcc006800 + ch * 0x14)); //  & 0xC0F;
		v &= v<<1;
		exi_handler_context[ch * 3 + EXI_EVENT_IRQ] = dev;
		if (v & 0x800)
			have_irq(ch * 3 + EXI_EVENT_INSERT);
		if (v & 8)
			have_irq(ch * 3 + EXI_EVENT_TC);
		if (v & 2)
			have_irq(ch * 3 + EXI_EVENT_IRQ);
		*(unsigned long*)(0xcc006800 + ch * 0x14) |= v;
	}

	if (retrig)
		trigger_interrupt(dev);
	spin_unlock(&gc_bba_lock);
	return IRQ_HANDLED;



//	struct netif *netif = (struct netif*)ct;
	int s;
	
	eth_exi_outb(2, 0);
	s = eth_exi_inb(3);
	
	if (s & 0x80)
	{
		eth_exi_outb(3, 0x80);
		gcif_service(dev);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x40)
	{
		printk("GCIF - EXI - 0x40!\n");
		eth_exi_outb(3, 0x40);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x20)
	{
		printk("GCIF - EXI - CMDERR!\n");
		eth_exi_outb(3, 0x20);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x10)
	{
		printk("GCIF - EXI - patchtru!\n");
		eth_exi_outb(3, 0x10);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x08)
	{
		printk("GCIF - EXI - HASH function\n");
		eth_exi_outb(3, 0x08);
		eth_exi_outb(2, 0xF8);
		return;
	}
	printk("GCIF - EXI - ?? %02x\n", s);
	eth_exi_outb(2, 0xF8);
	
	if (retrig)
		trigger_interrupt(dev);
	spin_unlock(&gc_bba_lock);
	return IRQ_HANDLED;
}


int __init gc_bba_probe(struct net_device *dev)
{
	int	i;
	static struct net_device_stats gc_bba_netstats;
	short s=0;
	long l;
	/*dev->priv = kmalloc(sizeof(struct net_device_stats), GFP_KERNEL);*/
	printk("gc_bba_probe\n");

	SET_MODULE_OWNER(dev);

	printk(KERN_INFO "%s: Nintendo GameCube broadband adapter", dev->name);

	/* probe for adapter */
	rx_page = 0;
	//select_nic();
	
	exi_select(0, 2, 5);
	exi_imm_ex(0, &s, 2, EXI_WRITE);
	exi_imm_ex(0, &l, 4, EXI_READ);
	exi_deselect(0);
	
	printk(": %u\n", l);
	if (l != 0x4020200) {
		printk("BBA not found");
		return -ENODEV;
	}
	

	printk("initializing BBA...\n");

	eth_outb(0x60, 0);	// unknown
	udelay(10000);
	eth_exi_inb_slow(0xF);
	udelay(10000);
	eth_outb(0, 1);	 // reset
	udelay(10000);
	eth_outb(0, 0);	 // /reset

	eth_exi_outs(4, "\xd1\x07\x75\x75", 2);
	eth_exi_outb(5, 0x4e);
	
	printk("BBA %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 eth_exi_inb(0), eth_exi_inb(1), eth_exi_inb(2), eth_exi_inb(3), 
		 eth_exi_inb(4), eth_exi_inb(5), eth_exi_inb(6), eth_exi_inb(7));

	eth_outb(0x5b, eth_inb(0x5b)&~(1<<7));
	eth_outb(0x5e, 1);
	eth_outb(0x5c, eth_inb(0x5c)|4);
	eth_outb(1, 0x11);
//	eth_outb(1, 0);
	eth_outb(0x50, 0x80);

	udelay(10000);
	
	// recvinit
	eth_outb(0xA,  0x1);
	eth_outb(0xB,  0x0);
	eth_outb(0x16, 0x1);
	eth_outb(0x17, 0x0);
	eth_outb(0x18, 0x1);
	eth_outb(0x19, 0x0);
	
	eth_outb(0x1a, 0xF);
	eth_outb(0x1b, 0);
	
	eth_outb(1, (eth_inb(1) & 0xFE) | 0x12);
	
	eth_outb(0, 8);
	eth_outb(0x32, 8);
	
	eth_ins(0x20, dev->dev_addr, ETH_LEN); 
	printk("MAC ADDRESS %02x:%02x:%02x:%02x:%02x:%02x\n", 
		dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2], 
		dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	/* Get the adapter ethernet address from the ROM */
	for (i = 0; i < ETH_ALEN; i++) {
		dev->broadcast[i] = 0xff;
	}

	eth_exi_outb(0x2, 0xFF);
	eth_exi_outb(0x3, 0xFF);
  
	eth_outb(8, 0xFF); // enable all IRQs
	eth_outb(9, 0xFF); // clear all irqs

	printk("after all: irq mask %x %x\n", eth_inb(8), eth_inb(9));

	/* Initialize the device structure. */
	dev->priv = &gc_bba_netstats;

	memset(dev->priv, 0, sizeof(struct net_device_stats));
	dev->get_stats = get_stats;

	dev->open = gc_bba_open;
	dev->stop = gc_bba_close;
	dev->hard_start_xmit = &gc_bba_start_xmit;


	ether_setup(dev);


	dev->flags&=~IFF_MULTICAST;

	return 0;
}

static int adapter_init(struct net_device *dev)
{
	int	i;

	//select_nic();
	rx_page = 0; /* used by RESET */

	printk("initializing BBA...\n");

	eth_outb(0x60, 0);	// unknown
	udelay(10000);
	eth_exi_inb_slow(0xF);
	udelay(10000);
	eth_outb(0, 1);	 // reset
	udelay(10000);
	eth_outb(0, 0);	 // /reset

	eth_exi_outs(4, "\xd1\x07\x75\x75", 2);
	eth_exi_outb(5, 0x4e);
	
	printk("BBA %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 eth_exi_inb(0), eth_exi_inb(1), eth_exi_inb(2), eth_exi_inb(3), 
		 eth_exi_inb(4), eth_exi_inb(5), eth_exi_inb(6), eth_exi_inb(7));
	eth_outb(0x5b, eth_inb(0x5b)&~(1<<7));
	eth_outb(0x5e, 1);
	eth_outb(0x5c, eth_inb(0x5c)|4);
	eth_outb(1, 0x11);
//	eth_outb(1, 0);
	eth_outb(0x50, 0x80);

	udelay(10000);
	
	// recvinit
	eth_outb(0xA,  0x1);
	eth_outb(0xB,  0x0);
	eth_outb(0x16, 0x1);
	eth_outb(0x17, 0x0);
	eth_outb(0x18, 0x1);
	eth_outb(0x19, 0x0);
	
	eth_outb(0x1a, 0xF);
	eth_outb(0x1b, 0);
	
	eth_outb(1, (eth_inb(1) & 0xFE) | 0x12);
	
	eth_outb(0, 8);
	eth_outb(0x32, 8);

	eth_ins(0x20, dev->dev_addr, ETH_LEN); 
	printk("MAC ADDRESS %02x:%02x:%02x:%02x:%02x:%02x\n", 
		dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2], 
		dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	/* Get the adapter ethernet address from the ROM */
	for (i = 0; i < ETH_ALEN; i++) {
		dev->broadcast[i] = 0xff;
	}
	
//	eth_ins(0x20, dev->ethaddr->addr, 6); 
//	printk("MAC ADDRESS %02x:%02x:%02x:%02x:%02x:%02x\n", 
//		gcif->ethaddr->addr[0], gcif->ethaddr->addr[1], gcif->ethaddr->addr[2], 
//		gcif->ethaddr->addr[3], gcif->ethaddr->addr[4], gcif->ethaddr->addr[5]);

        exi_request_irq(2, EXI_EVENT_IRQ, gcif_irq_handler, NULL);

	eth_exi_outb(0x2, 0xFF);
	eth_exi_outb(0x3, 0xFF);
  
	eth_outb(8, 0xFF); // enable all IRQs
	eth_outb(9, 0xFF); // clear all irqs

	printk("after all: irq mask %x %x\n", eth_inb(8), eth_inb(9));

	netif_start_queue(dev);

	return 0; /* OK */
}

static struct net_device gc_bba_dev;

static int __init gc_bba_init(void)
{
	printk("gc_bba_init\n");
	spin_lock_init(&gc_bba_lock);
	gc_bba_dev.init = gc_bba_probe;
	if (register_netdev(&gc_bba_dev) != 0)
		return -EIO;
	return 0;
}

static void __exit gc_bba_exit(void)
{
	printk("gc_bba_exit\n");
	unregister_netdev(&gc_bba_dev);
}

void gcif_irq_handler(int channel, int event, void *ct)
{
//	struct netif *netif = (struct netif*)ct;
	int s;
	struct net_device *dev = (struct net_device *)ct;
	
	eth_exi_outb(2, 0);
	s = eth_exi_inb(3);
	
	if (s & 0x80)
	{
		eth_exi_outb(3, 0x80);
		gcif_service(dev);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x40)
	{
		printk("GCIF - EXI - 0x40!\n");
		eth_exi_outb(3, 0x40);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x20)
	{
		printk("GCIF - EXI - CMDERR!\n");
		eth_exi_outb(3, 0x20);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x10)
	{
		printk("GCIF - EXI - patchtru!\n");
		eth_exi_outb(3, 0x10);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x08)
	{
		printk("GCIF - EXI - HASH function\n");
		eth_exi_outb(3, 0x08);
		eth_exi_outb(2, 0xF8);
		return;
	}
	printk("GCIF - EXI - ?? %02x\n", s);
	eth_exi_outb(2, 0xF8);
}


module_init(gc_bba_init);
module_exit(gc_bba_exit);

MODULE_AUTHOR("Stefan Esser <se@nopiracy.de>");
MODULE_DESCRIPTION("Nintendo GameCube BBA Ethernet driver");
MODULE_LICENSE("GPL");
