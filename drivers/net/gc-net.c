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

/*
 * $Id: gc-net.c,v 1.18 2004/02/26 22:33:51 hamtitampti Exp $
 *
 * $Log: gc-net.c,v $
 * Revision 1.18  2004/02/26 22:33:51  hamtitampti
 * 0% packet loss now :-)
 *
 * Revision 1.17  2004/02/11 20:15:27  hamtitampti
 * small changes, little bit better now
 *
 * Revision 1.16  2004/02/08 22:13:22  hamtitampti
 * added log tag
 *
 *
 */

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

#define BBA_DBG(format, arg...); { }
//#define BBA_DBG(format, arg...) printk(f,## arg)

//#define PACKETS_PER_IRQ (0x40)	// 2 packets / irq
#define PACKETS_PER_IRQ 0	// IRQ / packet

#define BBA_IRQ 4
#define IRQ_EXI 4
//#define PKT_BUF_SZ		1536	/* Size of each temporary Rx buffer. */

struct gc_private {
	int linkup;                     /* Link state */
	int linkspeed;                 /* Link speed */
	int fullduplex;                /* Duplex mode */
	int promisc;                   /* Promiscuous status */
	
	spinlock_t lock;
	spinlock_t phylock;             /* phy lock */
	unsigned long lockflags;        /* Lock flags */
    	
	// Statistical Data
	struct net_device_stats stats;
};



void gcif_irq_handler(int channel, int event, void *ct);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// must be MOVED LATER
///////////////////////////////////////////////////////////////////////////////////////////////////////

/* exi_select: enable chip select, set speed */

int selected = 0;
void exi_select(int channel, int device, int freq)
{
	volatile unsigned long *exi = (volatile unsigned long *)0xCC006800;
	selected ++;
	if (selected != 1)
		panic("-------- select while selected!\n");
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
	selected--;
	if (selected)
		panic("deselect broken!");
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
		exi_handler[t](t/3, t%3, exi_handler_context[t]);
		return 1;
	} else
	{
		BBA_DBG("UNHANDLED EXI\n");
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
		*(unsigned long*)(0xcc006800 + ch * 0x14) |= v;
		if (v & 0x800)
			have_irq(ch * 3 + EXI_EVENT_INSERT);
		if (v & 8)
			have_irq(ch * 3 + EXI_EVENT_TC);
		if (v & 2)
			have_irq(ch * 3 + EXI_EVENT_IRQ);
	}
}

void exi_init()
{
	BBA_DBG("EXI: init interrupts\n");
	//request_irq(IRQ_EXI, exi_interrupt_handler, 0);
	//request_debug_irq(IRQ_EXI, exi_interrupt_debug_handler, 0);

	*(unsigned long*)(0xcc006814) |= 3<<10; // enable&clear irq for insertion
}

int exi_request_irq(int channel, int event, exi_irq_handler_t *handler, void *context)
{
	if (exi_handler[channel * 3 + event])
	{
		BBA_DBG("EXI: irq %d:%d already used!\n", channel, event);
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


#define RUNT 60		/* Too small Ethernet packet */
#define ETH_LEN 6

/*
 * D-Link driver variables:
 */


#define GBA_IRQ_MASK	0xFF
#define GBA_RX_BP	0x1
#define GBA_RX_RWP	0x1
#define GBA_RX_RRP	0x1
#define GBA_RX_RHBP	0xf

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
	struct gc_private *priv = (struct gc_private *)dev->priv;
	BBA_DBG("gc_bba_open\n");

	int ret = request_irq(dev->irq, gc_bba_interrupt, 0, dev->name, dev);
	if (ret) {
		BBA_DBG(KERN_ERR "%s: unable to get IRQ %d\n", dev->name, dev->irq);
		return ret;
	}

	spin_lock_irqsave(&priv->lock, priv->lockflags);
	ret = adapter_init(dev);
	spin_unlock_irqrestore(&priv->lock, priv->lockflags);
	netif_start_queue(dev);
	
	return ret;
}

/*
 * The inverse routine to de600_open().
 */


static int gc_bba_close(struct net_device *dev)
{
	BBA_DBG("gc_bba_close\n");

//	//select_nic();
//	de600_put_command(RESET);
//	de600_put_command(STOP_RESET);
//	de600_put_command(0);
//	//select_prn();
	free_irq(dev->irq, dev);
	return 0;
}


static struct net_device_stats *gc_stats(struct net_device *dev)
{
	struct gc_private *priv = (struct gc_private *)dev->priv;
	return &priv->stats;
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
	struct gc_private *priv = (struct gc_private *)dev->priv;
	unsigned char Reg0;
	
	spin_lock_irqsave(&priv->lock, priv->lockflags);
	netif_stop_queue(dev);
	// TX Fifo Page to 0
	eth_outb(0xf, 0);
	eth_outb(0xe, 0);
	
	//printk("XMIT packet : len %d\n",skb->len);
	/*
		We send using the FIFO mode
	*/
	/* Start real output */
	BBA_DBG("gc_bba_start_xmit:len=%d, page %d/%d\n", skb->len, tx_fifo_in, free_tx_pages);

	// We set the packetbuffer to beginning...
	
	unsigned int val=0xC0004800;	// register 0x48 is the output queue

	exi_select(0, 2, 5);
	exi_imm(0, &val, 4, EXI_WRITE, 0);
	exi_sync(0);

	// Send Data from skb buffer to Network Driver now
	exi_imm_ex(0, skb->data, skb->len, EXI_WRITE);	

	/*
	The Network Drivers wants a minium of 60bytes to be filled into,
	so we check if we have at leaset 60 bytes
	*/

	if ( skb->len < RUNT) {
		char buf0[60+4];
		memset(buf0, 0, 60+4);
		exi_imm_ex(0, buf0, RUNT - skb->len, EXI_WRITE);
	}

	exi_deselect(0);

	// Start Hard Send
	val = eth_inb(0);
	val |=4;
	eth_outb(0, val);


	spin_unlock_irqrestore(&priv->lock, priv->lockflags);
	dev->trans_start = jiffies;
	dev_kfree_skb(skb);
	 	
	priv->stats.tx_bytes++;
 	priv->stats.tx_packets++;

	return 0;
}


static void gc_input(struct net_device *dev)
{

	struct gc_private *priv = (struct gc_private *)dev->priv;
	struct sk_buff	*skb;

	unsigned short size, p_read, p_write;
	unsigned short next_receive_frame;
	
	int ptr;
	
	/*
	Input Flow concept: Take a look to Hardware spec page 35
	*/
	
	eth_outb(0x3a, 2);
	if (eth_inb(0x3a) & 2)
	{
		BBA_DBG("NO DATA AVAILABLE!\n");
		priv->stats.rx_missed_errors++;
		return ;
	}
		
	/*
		Receive Buffer Write Page Pointer: Current receive write page pointer. The MSB
		is the Reg17h.3 bit. The LSB is the Reg16h.0 bit. This register is controlled by
		GMAC only. An internal Byte Counter (RWPBC) is associated with this page
		register.
	*/
	p_write  = eth_inb(0x16);
	p_write |= (eth_inb(0x17)&0x0f) << 8;

	/*
		Receive buffer Read Page Pointer: Current receive read page pointer. RRP[11:0]
		is mapped to MA[19:8]. The MSB is the Reg19h.3 bit. The LSB is the Reg18h.0
		bit. This register is normally controlled by the device driver. An internal Byte
		Counter (RRPBC) is associated with this page register.
	*/
	
	p_read  = eth_inb(0x18);
	p_read |= (eth_inb(0x19)&0x0f) << 8;
	
	if (p_read == p_write) return;
	
	//printk("Received a packet .. (start)\n");
	
	BBA_DBG("Receive Buffer Page Pointer: %x\n", p_read);

	unsigned char descr[4];

	size=0;

	eth_ins(p_read << 8, descr, 4);

	next_receive_frame  = descr[0];
	next_receive_frame |= (descr[1] & 0x0f) << 8;
	
	if (next_receive_frame>0xf) next_receive_frame = 1;

	eth_outb(0x18, next_receive_frame&0xff);
	eth_outb(0x19, (next_receive_frame&0x0f00)>>8);
	
		
	/*
		Size Looks Crazy, but ok, the Packet Lenght is indeed 3 nibbles = 12 bits
		and shifed with 4 bit to the top.
	*/	
	
	size = (descr[1] & 0xf0);	/* low byte */
	size |= (descr[2] << 8);	/* high byte */
	size = size >>4;
	size -= 4;			/* Ignore trailing 4 CRC-bytes */
	
	// Max of 6*256 (pages * pageSize) can happen

	if ((size < 32)  ||  (size > 1536)) {
		BBA_DBG(KERN_WARNING "%s: Bogus packet size %d.\n", dev->name, size);
		if (size > 10000) {
			adapter_init(dev);
			priv->stats.rx_length_errors++;
		}
		return;
	}

	// We allocate Space for the Kernel IP System now
	skb = dev_alloc_skb(size+2);
	
	if (!skb) {
		BBA_DBG("%s: Couldn't allocate a sk_buff of size %d.\n", dev->name, size);
		priv->stats.rx_errors++;
		return;
	}
	
	skb->dev = dev;
	skb_reserve(skb,2);	/* Align */

	/* 'skb->data' points to the start of sk_buff data area. */
	skb_put(skb,size);

	// We calculate the DMA position
	ptr = (p_read << 8) + 4;
	//printk("Packet: %d %d\n",p_read,size);
	
	if ((ptr + size) < ((GBA_RX_RHBP+1)<<8)) {
		// Full packet is linear to read
		eth_ins(ptr, skb->data, size);
		skb->protocol=eth_type_trans(skb,dev);
	} else {
		int temp_size = ((GBA_RX_RHBP+1)<<8) - ptr;
		
		// Full packet is Fragmentet to read
		eth_ins(ptr, skb->data, temp_size);

		p_read = GBA_RX_BP;
		
		eth_ins(p_read<<8, &skb->data[temp_size], size-temp_size);
		skb->protocol=eth_type_trans(skb,dev);		
		
	}

	
	/*
		We update the  Read Page Pointer with the next pointer, which was given to us
	*/

	netif_rx(skb);

	/* update stats */
	dev->last_rx = jiffies;

	priv->stats.rx_packets ++;	/* count all receives */
	priv->stats.rx_bytes ++;	/* count all received bytes */
	/*
	next_receive_frame  = descr[0];
	next_receive_frame |= (descr[1] & 0x0f) << 8;
	
	printk("Current Framepointer: %d\n",p_read);
	printk("Next    Framepointer: %d\n",next_receive_frame);
	printk("Write   Framepointer: %d\n",p_write);
	printk("Recieved Len: %d\n",skb->len);
	
	if (next_receive_frame != p_write) printk("Multipacket Seen\n");
	if (next_receive_frame != p_write) gc_input(dev);
	*/
	gc_input(dev);
	
	return ;
}


static void inline gcif_service(struct net_device *dev)
{

	struct gc_private *priv = (struct gc_private *)dev->priv;

	int inb9 = eth_inb(9);
	int inb8 = eth_inb(8);

	int status = inb9 & inb8;

	BBA_DBG("gcif_service: %08x %08x status %08x\n", inb8, inb9, status);

	if (!status) {
		eth_outb(9, 0xff);
		BBA_DBG("?? GC irq but no irq ??\n");
	}

	if (status & 4)
	{
		eth_outb(9, 4);
		// We clear the IRQ
		netif_wake_queue(dev);		/* allow more packets into adapter */

		
		// TX Transmisstion compleated ...

		/*
		int s = eth_inb(4);
		if (s)   // should not occur, since 4 == TX OK
			BBA_DBG("tx error %02x\n", s);
		*/
	}

	if (status & 2)
	{
		// We clear the IRQ
		eth_outb(9, 2);
		spin_lock_irqsave(&priv->lock, priv->lockflags);
		gc_input(dev);
		spin_unlock_irqrestore(&priv->lock, priv->lockflags);
		
	}

	if (status & 8)
	{
		eth_outb(9, 8);
		//printk("receive error :(\n");
	}

	if (status & 0x10)
	{
		eth_outb(9, 0x10);
		netif_wake_queue(dev);		/* allow more packets into adapter */
		//printk("tx error\n");
	}
	if (status & 0x20)
	{
		eth_outb(9, 0x20);
		BBA_DBG("rx fifo error\n");
	}
	if (status & 0x80)
	{
		eth_outb(9, 0x80);
		BBA_DBG("rx overflow!\n");
		gc_input(dev);
		
		// RWP
		eth_outb(0x16, GBA_RX_RWP);
		eth_outb(0x17, 0x0);
	
		// RRP
		eth_outb(0x18, GBA_RX_RRP);
		eth_outb(0x19, 0x0);
	
		// RHBP
		eth_outb(0x1a, GBA_RX_RHBP);
		eth_outb(0x1b, 0);
		

	}

	if (status & ~(0xBE))
	{
		eth_outb(9, status & ~0xBE);
		BBA_DBG("status %02x\n", status & ~0xBE);
	}
//	eth_outb(9, status);
}


/*
 * The typical workload of the driver:
 * Handle the network interface interrupts.
 */

static irqreturn_t gc_bba_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	
	struct net_device *dev  = (struct net_device *)dev_id;
    	struct gc_private *priv = (struct gc_private *)dev->priv;

	int		retrig = 0;

	/* This might just as well be deleted now, no crummy drivers present :-) */
	if ((dev == NULL) || (dev->irq != irq)) {
		BBA_DBG(KERN_ERR "%s: bogus interrupt %d\n", dev?dev->name:"GC_BBA", irq);
		return IRQ_NONE;
	}

	spin_lock(&priv->lock);
	
	int ch;
	for (ch = 0; ch < 3; ++ch)
	{
		unsigned long v = (*(unsigned long*)(0xcc006800 + ch * 0x14)); //  & 0xC0F;
		v &= v<<1;
		*(unsigned long*)(0xcc006800 + ch * 0x14) |= v;
		exi_handler_context[ch * 3 + EXI_EVENT_IRQ] = dev;
		if (v & 0x800)
			have_irq(ch * 3 + EXI_EVENT_INSERT);
		if (v & 8)
			have_irq(ch * 3 + EXI_EVENT_TC);
		if (v & 2)
			have_irq(ch * 3 + EXI_EVENT_IRQ);
	}

	if (retrig)
		trigger_interrupt(dev);

	spin_unlock(&priv->lock);
	
	return IRQ_HANDLED;

}


int __init gc_bba_probe(struct net_device *dev)
{
	struct gc_private *priv;
	int	i;
	short s=0;
	long l;


	BBA_DBG("gc_bba_probe\n");

	
	SET_MODULE_OWNER(dev);


	//select_nic();

	exi_select(0, 2, 5);
	exi_imm_ex(0, &s, 2, EXI_WRITE);
	exi_imm_ex(0, &l, 4, EXI_READ);
	exi_deselect(0);

	BBA_DBG(": %u\n", l);
	if (l != 0x4020200) {
		BBA_DBG("GameCube broadband adapter not found");
		return -ENODEV;
	}


	BBA_DBG("initializing BBA...\n");

	eth_outb(0x60, 0);	// unknown
	udelay(10000);
	eth_exi_inb_slow(0xF);
	udelay(10000);
	eth_outb(0, 1);	 // reset
	udelay(10000);
	eth_outb(0, 0);	 // /reset

	eth_exi_outs(4, "\xd1\x07\x75\x75", 2);
	eth_exi_outb(5, 0x4e);

//	BBA_DBG("BBA %02x %02x %02x %02x %02x %02x %02x %02x\n",
//		 eth_exi_inb(0), eth_exi_inb(1), eth_exi_inb(2), eth_exi_inb(3),
//		 eth_exi_inb(4), eth_exi_inb(5), eth_exi_inb(6), eth_exi_inb(7));

	eth_outb(0x5b, eth_inb(0x5b)&~(1<<7));
	eth_outb(0x5e, 1);
	eth_outb(0x5c, eth_inb(0x5c)|4);
	eth_outb(1, 0x11 | PACKETS_PER_IRQ);

	eth_outb(0x50, 0x80);

	udelay(10000);

	

	// BP
	eth_outb(0xA, GBA_RX_BP);
	eth_outb(0xB, 0x0);

	// RWP
	eth_outb(0x16, GBA_RX_RWP);
	eth_outb(0x17, 0x0);
	
	// RRP
	eth_outb(0x18, GBA_RX_RRP);
	eth_outb(0x19, 0x0);
	
	// RHBP
	eth_outb(0x1a, GBA_RX_RHBP);
	eth_outb(0x1b, 0);

	eth_outb(1, (eth_inb(1) & 0xFE) | 0x12| PACKETS_PER_IRQ);

	eth_outb(0, 8);
	eth_outb(0x32, 8);

	eth_ins(0x20, dev->dev_addr, ETH_LEN);

	BBA_DBG(KERN_INFO "%s: Nintendo GameCube broadband adapter", dev->name);
	BBA_DBG(", %02x:%02x:%02x:%02x:%02x:%02x.\n",
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

	BBA_DBG("after all: irq mask %x %x\n", eth_inb(8), eth_inb(9));

	/* Initialize the device structure. */


	dev->priv = (struct gc_private *)kmalloc(sizeof(struct net_device_stats), GFP_KERNEL);
	priv = (struct gc_private *)dev->priv;
	dev->irq = BBA_IRQ;
	
	
	dev->get_stats = gc_stats;

	dev->open = gc_bba_open;
	dev->stop = gc_bba_close;
	//&gc_bba_start_xmit ( i am confused)
	dev->hard_start_xmit = &gc_bba_start_xmit;
	
	spin_lock_init(&priv->lock);
	
	ether_setup(dev);

	dev->flags&=~IFF_MULTICAST;

	return 0;
}

static int adapter_init(struct net_device *dev)
{
	int	i;
	
	struct gc_private *priv = (struct gc_private *)dev->priv;
	
	BBA_DBG("initializing BBA...\n");

	
	
	eth_outb(0x60, 0);	// unknown
	udelay(10000);
	eth_exi_inb_slow(0xF);
	udelay(10000);
	eth_outb(0, 1);	 // reset
	udelay(10000);
	eth_outb(0, 0);	 // /reset

	eth_exi_outs(4, "\xd1\x07\x75\x75", 2);
	eth_exi_outb(5, 0x4e);

	BBA_DBG("BBA %02x %02x %02x %02x %02x %02x %02x %02x\n",
		 eth_exi_inb(0), eth_exi_inb(1), eth_exi_inb(2), eth_exi_inb(3),
		 eth_exi_inb(4), eth_exi_inb(5), eth_exi_inb(6), eth_exi_inb(7));

	eth_outb(0x5b, eth_inb(0x5b)&~(1<<7));
	eth_outb(0x5e, 1);
	eth_outb(0x5c, eth_inb(0x5c)|4);
	eth_outb(1, 0x11|PACKETS_PER_IRQ);

	eth_outb(0x50, 0x80);

	udelay(10000);

	
	// BP
	eth_outb(0xA, GBA_RX_BP);
	eth_outb(0xB, 0x0);

	// RWP
	eth_outb(0x16, GBA_RX_RWP);
	eth_outb(0x17, 0x0);
	
	// RRP
	eth_outb(0x18, GBA_RX_RRP);
	eth_outb(0x19, 0x0);
	
	// RHBP
	eth_outb(0x1a, GBA_RX_RHBP);
	eth_outb(0x1b, 0);
	
	eth_outb(1, (eth_inb(1) & 0xFE) | 0x12| PACKETS_PER_IRQ);

	eth_outb(0, 8);
	eth_outb(0x32, 8);

	eth_ins(0x20, dev->dev_addr, ETH_LEN);

	BBA_DBG("MAC ADDRESS %02x:%02x:%02x:%02x:%02x:%02x\n",
		dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
		dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

	/* Get the adapter ethernet address from the ROM */
	for (i = 0; i < ETH_ALEN; i++) {
		dev->broadcast[i] = 0xff;
	}

//	eth_ins(0x20, dev->ethaddr->addr, 6);
	BBA_DBG("MAC ADDRESS %02x:%02x:%02x:%02x:%02x:%02x\n",
		gcif->ethaddr->addr[0], gcif->ethaddr->addr[1], gcif->ethaddr->addr[2],
		gcif->ethaddr->addr[3], gcif->ethaddr->addr[4], gcif->ethaddr->addr[5]);
	/*
	strncpy(dev->name,"GameCube BBA",IFNAMSIZ);
	dev->name[IFNAMSIZ-1]= 0;
	*/
	
        exi_request_irq(2, EXI_EVENT_IRQ, gcif_irq_handler, NULL);

	eth_exi_outb(0x2, 0xFF);
	eth_exi_outb(0x3, 0xFF);

	eth_outb(8, 0xFF); // enable all IRQs
	eth_outb(9, 0xFF); // clear all irqs

//	eth_outb(0x30, (eth_inb(0x30) | 0x2));	// 100 Mbit ?

	
	BBA_DBG("after all: irq mask %x %x\n", eth_inb(8), eth_inb(9));

	return 0; /* OK */
}

static struct net_device gc_bba_dev;

static int __init gc_bba_init(void)
{
	memset(&gc_bba_dev,0,sizeof(struct net_device));
	
	
	BBA_DBG("gc_bba_init\n");

//	exi_init();

	gc_bba_dev.init = gc_bba_probe;
	if (register_netdev(&gc_bba_dev) != 0)
		return -EIO;
	return 0;
}

static void __exit gc_bba_exit(void)
{
	BBA_DBG("gc_bba_exit\n");
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
//		BBA_DBG("GC_IRQ service.\n");
		eth_exi_outb(3, 0x80);
		gcif_service(dev);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x40)
	{
		eth_exi_outb(3, 0x40);
		eth_exi_outb(2, 0xF8);
//		BBA_DBG("GCIF - EXI - 0x40!\n");
		adapter_init(dev);
		return;
		
//		return;
	}
	if (s & 0x20)
	{
		BBA_DBG("GCIF - EXI - CMDERR!\n");
		eth_exi_outb(3, 0x20);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x10)
	{
		BBA_DBG("GCIF - EXI - patchtru!\n");
		eth_exi_outb(3, 0x10);
		eth_exi_outb(2, 0xF8);
		return;
	}
	if (s & 0x08)
	{
		BBA_DBG("GCIF - EXI - HASH function\n");
		eth_exi_outb(3, 0x08);
		eth_exi_outb(2, 0xF8);
		return;
	}
//	printk("GCIF - EXI - ?? %02x\n", s);
	eth_exi_outb(2, 0xF8);
}


module_init(gc_bba_init);
module_exit(gc_bba_exit);

MODULE_AUTHOR("Stefan Esser <se@nopiracy.de>");
MODULE_DESCRIPTION("Nintendo GameCube BBA Ethernet driver");
MODULE_LICENSE("GPL");
