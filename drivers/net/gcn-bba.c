/*
 * drivers/net/gcn-bba.c
 *
 * Nintendo GameCube Broadband Adapter driver
 * Copyright (C) 2004-2005 The GameCube Linux Team
 * Copyright (C) 2004,2005 Albert Herranz,Todd Jeffreys
 *
 * Based on previous work by Stefan Esser, Franz Lehner, Costis and tmbinc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

//#define BBA_DEBUG

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/inet.h>
#include <linux/sched.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <linux/exi.h>

#ifdef BBA_DEBUG
#  define DPRINTK(fmt, args...) \
          printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif


/*
 * Macronix mx98728ec supporting bits.
 *
 */

#define BBA_NCRA 0x00		/* Network Control Register A, RW */
#define   BBA_NCRA_RESET     (1<<0)	/* RESET */
#define   BBA_NCRA_ST0       (1<<1)	/* ST0, Start transmit command/status */
#define   BBA_NCRA_ST1       (1<<2)	/* ST1,  " */
#define   BBA_NCRA_SR        (1<<3)	/* SR, Start Receive */

#define BBA_NCRB 0x01		/* Network Control Register B, RW */
#define   BBA_NCRB_PR        (1<<0)	/* PR, Promiscuous Mode */
#define   BBA_NCRB_CA        (1<<1)	/* CA, Capture Effect Mode */
#define   BBA_NCRB_PM        (1<<2)	/* PM, Pass Multicast */
#define   BBA_NCRB_PB        (1<<3)	/* PB, Pass Bad Frame */
#define   BBA_NCRB_AB        (1<<4)	/* AB, Accept Broadcast */
#define   BBA_NCRB_HBD       (1<<5)	/* HBD, reserved */
#define   BBA_NCRB_RXINTC0   (1<<6)	/* RXINTC, Receive Interrupt Counter */
#define   BBA_NCRB_RXINTC1   (1<<7)	/*  " */
#define     BBA_NCRB_1_PACKET_PER_INT  (0<<6)	/* 0 0 */
#define     BBA_NCRB_2_PACKETS_PER_INT (1<<6)	/* 0 1 */
#define     BBA_NCRB_4_PACKETS_PER_INT (2<<6)	/* 1 0 */
#define     BBA_NCRB_8_PACKETS_PER_INT (3<<6)	/* 1 1 */

#define BBA_LTPS 0x04		/* Last Transmitted Packet Status, RO */
#define BBA_LRPS 0x05		/* Last Received Packet Status, RO */

#define BBA_IMR 0x08		/* Interrupt Mask Register, RW, 00h */
#define   BBA_IMR_FRAGIM     (1<<0)	/* FRAGIM, Fragment Counter Int Mask */
#define   BBA_IMR_RIM        (1<<1)	/* RIM, Receive Interrupt Mask */
#define   BBA_IMR_TIM        (1<<2)	/* TIM, Transmit Interrupt Mask */
#define   BBA_IMR_REIM       (1<<3)	/* REIM, Receive Error Interrupt Mask */
#define   BBA_IMR_TEIM       (1<<4)	/* TEIM, Transmit Error Interrupt Mask */
#define   BBA_IMR_FIFOEIM    (1<<5)	/* FIFOEIM, FIFO Error Interrupt Mask */
#define   BBA_IMR_BUSEIM     (1<<6)	/* BUSEIM, BUS Error Interrupt Mask */
#define   BBA_IMR_RBFIM      (1<<7)	/* RBFIM, RX Buffer Full Interrupt Mask */

#define BBA_IR 0x09		/* Interrupt Register, RW, 00h */
#define   BBA_IR_FRAGI       (1<<0)	/* FRAGI, Fragment Counter Interrupt */
#define   BBA_IR_RI          (1<<1)	/* RI, Receive Interrupt */
#define   BBA_IR_TI          (1<<2)	/* TI, Transmit Interrupt */
#define   BBA_IR_REI         (1<<3)	/* REI, Receive Error Interrupt */
#define   BBA_IR_TEI         (1<<4)	/* TEI, Transmit Error Interrupt */
#define   BBA_IR_FIFOEI      (1<<5)	/* FIFOEI, FIFO Error Interrupt */
#define   BBA_IR_BUSEI       (1<<6)	/* BUSEI, BUS Error Interrupt */
#define   BBA_IR_RBFI        (1<<7)	/* RBFI, RX Buffer Full Interrupt */

#define BBA_BP   0x0a/*+0x0b*/	/* Boundary Page Pointer Register */
#define BBA_TLBP 0x0c/*+0x0d*/	/* TX Low Boundary Page Pointer Register */
#define BBA_TWP  0x0e/*+0x0f*/	/* Transmit Buffer Write Page Pointer Register */
#define BBA_TRP  0x12/*+0x13*/	/* Transmit Buffer Read Page Pointer Register */
#define BBA_RWP  0x16/*+0x17*/	/* Receive Buffer Write Page Pointer Register */
#define BBA_RRP  0x18/*+0x19*/	/* Receive Buffer Read Page Pointer Register */
#define BBA_RHBP 0x1a/*+0x1b*/	/* Receive High Boundary Page Pointer Register */

#define BBA_RXINTT    0x14/*+0x15*/	/* Receive Interrupt Timer Register */

#define BBA_NAFR_PAR0 0x20	/* Physical Address Register Byte 0 */
#define BBA_NAFR_PAR1 0x21	/* Physical Address Register Byte 1 */
#define BBA_NAFR_PAR2 0x22	/* Physical Address Register Byte 2 */
#define BBA_NAFR_PAR3 0x23	/* Physical Address Register Byte 3 */
#define BBA_NAFR_PAR4 0x24	/* Physical Address Register Byte 4 */
#define BBA_NAFR_PAR5 0x25	/* Physical Address Register Byte 5 */

#define BBA_NWAYC 0x30		/* NWAY Configuration Register, RW, 84h */
#define   BBA_NWAYC_FD       (1<<0)	/* FD, Full Duplex Mode */
#define   BBA_NWAYC_PS100    (1<<1)	/* PS100/10, Port Select 100/10 */
#define   BBA_NWAYC_ANE      (1<<2)	/* ANE, Autonegotiation Enable */
#define   BBA_NWAYC_ANS_RA   (0x01<<3)	/* ANS, Restart Autonegotiation */
#define   BBA_NWAYC_LTE      (1<<7)	/* LTE, Link Test Enable */

#define BBA_GCA 0x32		/* GMAC Configuration A Register, RW, 00h */
#define   BBA_GCA_ARXERRB    (1<<3)	/* ARXERRB, Accept RX packet with error */

#define BBA_MISC 0x3d		/* MISC Control Register 1, RW, 3ch */
#define   BBA_MISC_BURSTDMA  (1<<0)
#define   BBA_MISC_DISLDMA   (1<<1)

#define BBA_TXFIFOCNT 0x3e/*0x3f*/	/* Transmit FIFO Counter Register */
#define BBA_WRTXFIFOD 0x48/*-0x4b*/	/* Write TX FIFO Data Port Register */

#define BBA_MISC2 0x50		/* MISC Control Register 2, RW, 00h */
#define   BBA_MISC2_HBRLEN0  (1<<0)	/* HBRLEN, Host Burst Read Length */
#define   BBA_MISC2_HBRLEN1  (1<<1)	/*  " */
#define   BBA_MISC2_AUTORCVR (1<<7)	/* Auto RX Full Recovery */

#define BBA_RX_STATUS_BF      (1<<0)
#define BBA_RX_STATUS_CRC     (1<<1)
#define BBA_RX_STATUS_FAE     (1<<2)
#define BBA_RX_STATUS_FO      (1<<3)
#define BBA_RX_STATUS_RW      (1<<4)
#define BBA_RX_STATUS_MF      (1<<5)
#define BBA_RX_STATUS_RF      (1<<6)
#define BBA_RX_STATUS_RERR    (1<<7)

#define BBA_TX_STATUS_CC0     (1<<0)
#define BBA_TX_STATUS_CC1     (1<<1)
#define BBA_TX_STATUS_CC2     (1<<2)
#define BBA_TX_STATUS_CC3     (1<<3)
#define  BBA_TX_STATUS_CCMASK (0x0f)
#define BBA_TX_STATUS_CRSLOST (1<<4)
#define BBA_TX_STATUS_UF      (1<<5)
#define BBA_TX_STATUS_OWC     (1<<6)
#define BBA_TX_STATUS_OWN     (1<<7)
#define BBA_TX_STATUS_TERR    (1<<7)

#define BBA_TX_MAX_PACKET_SIZE 1518	/* 14+1500+4 */
#define BBA_RX_MAX_PACKET_SIZE 1536	/* 6 pages * 256 bytes */


/*
 * EXpansion Interface glue for the Broadband Adapter.
 *
 */
#define BBA_EXI_ID 0x04020200

#define BBA_EXI_CHANNEL_IRQ 2 /* INT line uses EXI2INTB */
#define BBA_EXI_CHANNEL     0 /* rest of lines use EXI0xxx */
#define BBA_EXI_DEVICE      2 /* chip select, EXI0CSB2 */
#define BBA_EXI_FREQ        5 /* 32MHz */

#define BBA_CMD_IR_MASKALL  0x00
#define BBA_CMD_IR_MASKNONE 0xf8

struct bba_private {
	//spinlock_t lock;

	u32 msg_enable;
	u8 revid;
	u8 __0x04_init[2];
	u8 __0x05_init;

	struct exi_device *exi_dev;
	struct net_device *dev;
	struct net_device_stats stats;
	struct task_struct *interrupt_thread;
	wait_queue_head_t wait_queue;
	atomic_t num_interrupts;
	struct sk_buff *skb;
	void *dma_send_ptr;
	void *dma_recv_ptr;
	u8 dma_send_buffer[ETH_FRAME_LEN + EXI_DMA_ALIGNMENT];
	u8 dma_recv_buffer[ETH_FRAME_LEN + EXI_DMA_ALIGNMENT];
};


static void bba_cmd_ins(struct bba_private *priv,int reg, void *val, int len);
static void bba_cmd_outs(struct bba_private *priv,int reg, void *val, int len);
static void bba_ins(struct bba_private *priv,int reg, void *val, int len);
static void bba_outs(struct bba_private *priv,int reg, void *val, int len);

static inline void *align(void *ptr) 
{
	u32 addr = (u32)ptr;
	return (void*)
		((addr + EXI_DMA_ALIGNMENT - 1) & ~(EXI_DMA_ALIGNMENT-1));
}

struct sleep_t
{
	volatile int done;
	wait_queue_head_t queue;
};

static void wakeup_callback(struct exi_command *cmd)
{
	struct sleep_t *val = (struct sleep_t *)cmd->param;
	/* set to true */
	val->done = 1;
	wake_up(&val->queue);
}

static void bba_do_sleeping_cmd(struct exi_command_group *cmd)
{
	int i;
	struct sleep_t sleep;

	sleep.done = 0;
	init_waitqueue_head(&sleep.queue);

	/* setup completion routines */
	for (i=0;i<cmd->num_commands-1;++i)
	{
		cmd->commands[i].completion_routine = NULL;
	}
	cmd->commands[i].param = (void*)&sleep;
	cmd->commands[i].completion_routine = wakeup_callback;
	/* perform the actions */
	exi_add_command_group(cmd,1);
	/* wait for completion */
	wait_event(sleep.queue,sleep.done);
}

static u32 bba_exi_probe(struct exi_device *exi_dev)
{
	u16 write;
	u32 read;
	struct exi_command_group cmd;
	struct exi_command sub[2];
	
	write = 0;

	cmd.flags = 0;
	cmd.commands = sub;
	cmd.num_commands = 2;
	cmd.dev = exi_dev;
	
	sub[0].flags = EXI_CMD_WRITE;
	sub[0].data  = &write;
	sub[0].len   = sizeof(write);
	
	sub[1].flags = EXI_CMD_READ;
	sub[1].data  = &read;
	sub[1].len   = sizeof(read);

	bba_do_sleeping_cmd(&cmd);
	return read;
}

static inline void bba_build_subcmd_ins(struct exi_command *subcmd,
					int reg,u32 *cmdbuffer,
					void *data,unsigned int len)
{
	*cmdbuffer = (reg << 8) | 0x80000000;
	
	subcmd[0].flags = EXI_CMD_WRITE;
	subcmd[0].data  = cmdbuffer;
	subcmd[0].len   = 4;
	subcmd[0].completion_routine = NULL;
	
	subcmd[1].flags = EXI_CMD_READ;
	subcmd[1].data  = data;
	subcmd[1].len   = len;
	subcmd[1].completion_routine = NULL;
}

static inline void bba_build_subcmd_outs(struct exi_command *subcmd,
					 int reg,u32 *cmdbuffer,
					 void *data,unsigned int len)
{
	*cmdbuffer = (reg << 8) | 0xC0000000;
	
	subcmd[0].flags = EXI_CMD_WRITE;
	subcmd[0].data  = cmdbuffer;
	subcmd[0].len   = 4;
	subcmd[0].completion_routine = NULL;
	
	subcmd[1].flags = EXI_CMD_WRITE;
	subcmd[1].data  = data;
	subcmd[1].len   = len;
	subcmd[1].completion_routine = NULL;
}

static inline void bba_build_subcmd_cmd_ins(struct exi_command *subcmd,
					    int reg,u16 *cmdbuffer,
					    void *data,unsigned int len)
{
	*cmdbuffer = (reg << 8);
	
	subcmd[0].flags = EXI_CMD_WRITE;
	subcmd[0].data  = cmdbuffer;
	subcmd[0].len   = 2;
	subcmd[0].completion_routine = NULL;
	
	subcmd[1].flags = EXI_CMD_READ;
	subcmd[1].data  = data;
	subcmd[1].len   = len;
	subcmd[1].completion_routine = NULL;
}

static inline void bba_build_subcmd_cmd_outs(struct exi_command *subcmd,
					     int reg,u16 *cmdbuffer,
					     void *data,unsigned int len)
{
	*cmdbuffer = (reg << 8) | 0x4000;
	
	subcmd[0].flags = EXI_CMD_WRITE;
	subcmd[0].data  = cmdbuffer;
	subcmd[0].len   = 2;
	subcmd[0].completion_routine = NULL;
	
	subcmd[1].flags = EXI_CMD_WRITE;
	subcmd[1].data  = data;
	subcmd[1].len   = len;
	subcmd[1].completion_routine = NULL;
}


static void bba_cmd_ins(struct bba_private *bba,int reg,void *val,int len)
{
	u16 write;
	struct exi_command_group cmd;
	struct exi_command sub[2];
	
	cmd.flags = 0;
	cmd.commands = sub;
	cmd.num_commands = 2;
	cmd.dev = bba->exi_dev;
	
	bba_build_subcmd_cmd_ins(sub,reg,&write,val,len);
	
	bba_do_sleeping_cmd(&cmd);
}

static void bba_cmd_outs(struct bba_private *bba,int reg,void *val,int len)
{
	u16 write;
	struct exi_command_group cmd;
	struct exi_command sub[2];

	cmd.flags = 0;
	cmd.commands = sub;
	cmd.num_commands = 2;
	cmd.dev = bba->exi_dev;
	
	bba_build_subcmd_cmd_outs(sub,reg,&write,val,len);
	
	bba_do_sleeping_cmd(&cmd);
}

static void bba_ins(struct bba_private *bba,int reg, void *val, int len)
{
	u32 write;
	struct exi_command_group cmd;
	struct exi_command sub[2];
	
	cmd.flags = 0;
	cmd.commands = sub;
	cmd.num_commands = 2;
	cmd.dev = bba->exi_dev;
	
	bba_build_subcmd_ins(sub,reg,&write,val,len);
	
	bba_do_sleeping_cmd(&cmd);
}

static void bba_outs(struct bba_private *bba,int reg, void *val, int len)
{
	u32 write;
	struct exi_command_group cmd;
	struct exi_command sub[2];
	
	cmd.flags = 0;
	cmd.commands = sub;
	cmd.num_commands = 2;
	cmd.dev = bba->exi_dev;
	
	bba_build_subcmd_outs(sub,reg,&write,val,len);
	
	bba_do_sleeping_cmd(&cmd);
}

static void bba_outs_dma(struct bba_private *bba,int reg, void *val, int len)
{
	u32 write;
	u32 dma;
	u32 rem;
	struct exi_command_group cmd;
	struct exi_command sub[3];
	
#ifdef BBA_DEBUG
	if ((u32)val & (EXI_DMA_ALIGNMENT-1)) {
		printk(KERN_INFO "WARNING, bba_outs_dma called with a unaligned buffer, addr is %p,len is %u\n",val,len);
	}
	else if (len < EXI_DMA_ALIGNMENT) {
		printk(KERN_INFO "WARNING, bba_outs_dma called with a length < 32, %u\n",len);
	}
#endif
	cmd.flags = 0;
	cmd.commands = sub;
	cmd.dev = bba->exi_dev;
	
	rem = len & (EXI_DMA_ALIGNMENT-1);  /* this is the remainder */
	dma = len & ~(EXI_DMA_ALIGNMENT-1); /* this is the aligned counter */
	
	bba_build_subcmd_outs(sub,reg,&write,val,dma);
	
	if (rem) {
		/* if we don't hit the 32 byte DMA alignment, we need to 
		   write the remaining data in a non-dma fashion */
		cmd.num_commands = 3;
		
		sub[2].flags = EXI_CMD_WRITE;
		sub[2].data  = val + dma;
		sub[2].len   = rem;
	}
	else {
		cmd.num_commands = 2;
	}
	bba_do_sleeping_cmd(&cmd);
}

static void bba_ins_dma(struct bba_private *bba,int reg, void *val, int len)
{
	u32 write;
	u32 dma;
	u32 rem;
	struct exi_command_group cmd;
	struct exi_command sub[3];
	
#ifdef BBA_DEBUG
	if ((u32)val & (EXI_DMA_ALIGNMENT-1)) {
		printk(KERN_INFO "WARNING, bba_ins_dma called with a unaligned buffer, addr is %p,len is %u\n",val,len);
	}
	else if (len < EXI_DMA_ALIGNMENT) {
		printk(KERN_INFO "WARNING, bba_ins_dma called with a length < 32, %u\n",len);
	}
#endif
	cmd.flags = 0;
	cmd.commands = sub;
	cmd.dev = bba->exi_dev;
	
	rem = len & (EXI_DMA_ALIGNMENT-1); /* this is the remainder */
	dma = len & ~(EXI_DMA_ALIGNMENT-1);/* this is the aligned counter */
	
	bba_build_subcmd_ins(sub,reg,&write,val,dma);
	
	if (rem) {
		/* if we don't hit the 32 byte DMA alignment, we need to 
		   write the remaining data in a non-dma fashion */
		cmd.num_commands = 3;
		
		sub[2].flags = EXI_CMD_READ;
		sub[2].data  = val + dma;
		sub[2].len   = rem;
	}
	else {
		cmd.num_commands = 2;
	}
	bba_do_sleeping_cmd(&cmd);
}

static u8 bba_cmd_in8_slow(struct bba_private *bba,int reg)
{
	u16 write;
	u8 val;
	struct exi_command_group cmd;
	struct exi_command sub[2];

	cmd.flags = EXI_DESELECT_UDELAY;
	cmd.commands = sub;
	cmd.num_commands = 2;
	cmd.dev = bba->exi_dev;
	cmd.deselect_udelay = 200;

	bba_build_subcmd_cmd_ins(sub,reg,&write,&val,1);
	
	bba_do_sleeping_cmd(&cmd);
	return val;
}

static inline u8 bba_cmd_in8(struct bba_private *bba,int reg)
{
	u8 val;
	bba_cmd_ins(bba,reg,&val,sizeof(val));
	return val;
}

static inline void bba_cmd_out8(struct bba_private *bba,int reg, u8 val)
{
	bba_cmd_outs(bba,reg,&val,sizeof(val));
}

static inline u8 bba_in8(struct bba_private *bba,int reg)
{
	u8 val;
	bba_ins(bba,reg, &val, sizeof(val));
	return val;
}

static inline void bba_out8(struct bba_private *bba,int reg, u8 val)
{
	bba_outs(bba,reg, &val, sizeof(val));
}

static inline u16 bba_in16(struct bba_private *bba,int reg)
{
	u16 val;
	bba_ins(bba,reg, &val, sizeof(val));
	return le16_to_cpup(&val);
}

static inline void bba_out16(struct bba_private *bba,int reg, u16 val)
{
	cpu_to_le16s(&val);
	bba_outs(bba,reg, &val, sizeof(val));
}

#define bba_in12(bba,reg)      (bba_in16((bba),(reg)) & 0x0FFF)
#define bba_out12(bba,reg,val) (bba_out16((bba),(reg),(val) & 0x0FFF))

/**
 * Nintendo GameCube Broadband Adapter driver.
 *
 */

#define DRV_MODULE_NAME   "gcn-bba"
#define DRV_DESCRIPTION   "Nintendo GameCube Broadband Adapter driver"
#define DRV_AUTHOR        "Albert Herranz, Todd Jeffreys"

char bba_driver_name[] = DRV_MODULE_NAME;
char bba_driver_string[] = DRV_DESCRIPTION;
char bba_driver_version[] = "0.4 - tcj";
char bba_copyright[] = "Copyright (C) 2004 " DRV_AUTHOR;

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_LICENSE("GPL");

#define PFX DRV_MODULE_NAME ": "

#define bba_printk(level, format, arg...) \
        printk(level PFX format , ## arg)

/**
 *
 * DRIVER NOTES
 *
 * 1. Packet Memory organization
 *
 * rx: 15 pages of 256 bytes, 2 full sized packets only (6 pages each)
 * tx: through FIFO, not using packet memory
 *
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |1|2|3|4|5|6|7|8|9|A|B|C|D|E|F|
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * ^                           ^
 * |                           |
 * TLBP                        RHBP
 * BP
 *
 */

#define BBA_INIT_TLBP	0x00
#define BBA_INIT_BP	0x01
#define BBA_INIT_RHBP	0x0f
#define BBA_INIT_RWP	BBA_INIT_BP
#define BBA_INIT_RRP	BBA_INIT_BP

#if defined(__BIG_ENDIAN_BITFIELD)
#define X(a,b)  b,a
#else
#define X(a,b)  a,b
#endif

struct bba_descr {
	u32 X(X(next_packet_ptr:12, packet_len:12), status:8);
} __attribute((packed));


static int bba_open(struct net_device *dev);
static int bba_close(struct net_device *dev);
static struct net_device_stats *bba_get_stats(struct net_device *dev);

static int bba_probe(struct exi_device *dev);
static void bba_remove(struct exi_device *dev);
static int bba_init_one(struct exi_device *dev);
static int __init bba_init_module(void);
static void __exit bba_exit_module(void);
static int bba_event_handler(int channel, void *dev0);

static int bba_reset(struct net_device *dev);

/**
 *
 */
static int bba_open(struct net_device *dev)
{
	int ret;
	
	/* according to patents, INTs will be triggered on EXI channel 2 */
	ret = exi_register_irq(BBA_EXI_CHANNEL_IRQ,bba_event_handler, dev);
	if (ret < 0) {
		bba_printk(KERN_ERR, "unable to register EXI IRQ for channel"
			   " %d\n",BBA_EXI_CHANNEL_IRQ);
		return ret;
	}

	ret = bba_reset(dev);

	netif_start_queue(dev);

	return ret;
}

/**
 *
 */
static int bba_close(struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;

	netif_carrier_off(dev);

	/* stop queue */
	netif_stop_queue(dev);

	/* stop receiver */
	bba_out8(priv,BBA_NCRA, bba_in8(priv,BBA_NCRA) & ~BBA_NCRA_SR);

	/* mask all interrupts */
	bba_out8(priv,BBA_IMR, 0x00);

	/* unregister exi event */
	exi_unregister_irq(BBA_EXI_CHANNEL_IRQ);
	return 0;
}

/**
 *
 */
static struct net_device_stats *bba_get_stats(struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	return &priv->stats;
}

static void bba_do_xmit(struct bba_private *priv)
{
	u32 packetLen;
	u8 ncra;
	void *dma_send_ptr;
        
        /* if the TXFIFO is in use we'll try it later when free */
	if ((ncra=bba_in8(priv,BBA_NCRA)) & (BBA_NCRA_ST0 | BBA_NCRA_ST1)) {
		return;
	}
	
	/* copy to the dma buffer if < ETH_ZLEN or unaligned */
	if ((priv->skb->len < ETH_ZLEN) || 
	    ((u32)priv->skb->data & (EXI_DMA_ALIGNMENT-1))) {
		memcpy(priv->dma_send_ptr,priv->skb->data,priv->skb->len);
		dma_send_ptr   = priv->dma_send_ptr;
		packetLen      = max(priv->skb->len,(u32)ETH_ZLEN);
	}
	else {			/* skb is perfectly aligned */
		dma_send_ptr   = priv->skb->data;
		packetLen      = priv->skb->len;
	}
	
	/* tell the card about the length of this packet */
	bba_out12(priv,BBA_TXFIFOCNT, priv->skb->len);

	/* store the packet in the TXFIFO */
	bba_outs_dma(priv,BBA_WRTXFIFOD,dma_send_ptr,packetLen);
	
	/* tell the card to send the packet right now */
	bba_out8(priv,BBA_NCRA,(ncra | BBA_NCRA_ST1) & ~BBA_NCRA_ST0);
	
	priv->dev->trans_start = jiffies;
	priv->stats.tx_bytes += priv->skb->len;
	priv->stats.tx_packets++;
	
	/* free the skb */
	dev_kfree_skb(priv->skb);
	priv->skb = NULL;
}
/**
 *
 */
static int bba_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	
	/* check if we already have something queued */
	if (priv->skb) {
		netif_stop_queue(dev);
		return NETDEV_TX_BUSY;
	}
	/* we are not able to send packets greater than this */
	if (skb->len > BBA_TX_MAX_PACKET_SIZE) {
		dev_kfree_skb(skb);
		priv->stats.tx_dropped++;
		return NETDEV_TX_BUSY;
	}

	/* store the skb */
	priv->skb = skb;
	/* stop the queue  */
	netif_stop_queue(dev);
	/* wake up the kernel thread to do it */
	wake_up(&priv->wait_queue);
	
	return NETDEV_TX_OK;
}

/**
 *
 */
static int bba_rx_err(u8 status, struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	int errors = 0;

	if (status & BBA_RX_STATUS_RERR) {
		errors++;
		priv->stats.rx_errors++;
		if (status & BBA_RX_STATUS_BF) {
			priv->stats.rx_over_errors++;
		}
		if (status & BBA_RX_STATUS_CRC) {
			priv->stats.rx_crc_errors++;
		}
		if (status & BBA_RX_STATUS_FO) {
			priv->stats.rx_fifo_errors++;
		}
		if (status & BBA_RX_STATUS_RW) {
			priv->stats.rx_length_errors++;
		}
		if (status & BBA_RX_STATUS_FAE) {
			priv->stats.rx_frame_errors++;
		}
	}
	if (status & BBA_RX_STATUS_RF) {
		priv->stats.rx_length_errors++;
	}

	if (errors) {
		if (netif_msg_rx_err(priv)) {
			bba_printk(KERN_DEBUG, "rx errors, status %8.8x.\n",
				   status);
		}

		/* stop receiver */
		bba_out8(priv,BBA_NCRA, bba_in8(priv,BBA_NCRA) & ~BBA_NCRA_SR);

		/* initialize page pointers */
		bba_out12(priv,BBA_RWP, BBA_INIT_RWP);
		bba_out12(priv,BBA_RRP, BBA_INIT_RRP);

		/* start receiver again */
		bba_out8(priv,BBA_NCRA, bba_in8(priv,BBA_NCRA) | BBA_NCRA_SR);
	}
	return errors;
}

/**
 *
 */
static int bba_rx(struct net_device *dev, int budget)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	struct sk_buff *skb;
	struct bba_descr descr;
	int status, size;
	unsigned long pos, top;
	unsigned short rrp, rwp;
	int received = 0;

	/* get current receiver pointers */
	rwp = bba_in12(priv,BBA_RWP);
	rrp = bba_in12(priv,BBA_RRP);

	while (netif_running(dev) && received < budget && rrp != rwp) {
		bba_ins(priv,rrp << 8, &descr, sizeof(descr));
		le32_to_cpus((u32 *) & descr);

		size = descr.packet_len - 4;	/* ignore CRC */
		status = descr.status;

		/* abort processing in case of errors */
		if (size > BBA_RX_MAX_PACKET_SIZE + 4
		    || (status & (BBA_RX_STATUS_RERR | BBA_RX_STATUS_FAE))) {
			bba_rx_err(status, dev);
			continue;
		}

		/* allocate a buffer, omitting the CRC (4 bytes) */
		skb = dev_alloc_skb(size + 2 + EXI_DMA_ALIGNMENT);
		if (!skb) {
			priv->stats.rx_dropped++;
			continue;
		}

		skb->dev = dev;
		skb_reserve(skb, EXI_DMA_ALIGNMENT - 2 - ETH_HLEN);/* align */
		skb_put(skb, size);

		pos = (rrp << 8) + 4;	/* skip descriptor */
		top = (BBA_INIT_RHBP + 1) << 8;

		if ((pos + size) < top) {
			/* full packet in one chunk, guaranteed to be > 32 */
			bba_ins_dma(priv,pos, skb->data, size);
		} else {
			/* packet wrapped */
			int chunk_size = top - pos;
			int rem = size - chunk_size;
			/* the first read, check for minimum dma length */
			if (chunk_size < EXI_DMA_ALIGNMENT) {
				bba_ins(priv,pos,skb->data,chunk_size);
			}
			else {
				bba_ins_dma(priv,pos,skb->data,chunk_size);
			}
			/* the second read, check for minimum dma length */
			rrp = BBA_INIT_RRP;
			if (rem < EXI_DMA_ALIGNMENT) {
				bba_ins(priv,rrp << 8,
					skb->data+chunk_size,rem);
			}
			else {
				/* load the rest into our dma buffer */
				bba_ins_dma(priv,rrp << 8,
					    priv->dma_recv_ptr,rem);
				/* now copy manually */
				memcpy(skb->data+chunk_size,
				       priv->dma_recv_ptr,rem);
			}
		}

		skb->protocol = eth_type_trans(skb, dev);

		dev->last_rx = jiffies;
		priv->stats.rx_bytes += size;
		priv->stats.rx_packets++;

		netif_rx(skb);
		received++;

		bba_out12(priv,BBA_RRP, descr.next_packet_ptr);

		rwp = bba_in12(priv,BBA_RWP);
		rrp = bba_in12(priv,BBA_RRP);
	}

	return received;
}

/**
 *
 */
static int bba_tx_err(u8 status, struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	int errors = 0;

	if (status & BBA_TX_STATUS_TERR) {
		errors++;
		priv->stats.tx_errors++;
		if (status & BBA_TX_STATUS_CCMASK) {
			priv->stats.collisions +=
			    (status & BBA_TX_STATUS_CCMASK);
		}
		if (status & BBA_TX_STATUS_CRSLOST) {
			priv->stats.tx_carrier_errors++;
		}
		if (status & BBA_TX_STATUS_UF) {
			priv->stats.tx_fifo_errors++;
		}
		if (status & BBA_TX_STATUS_OWC) {
			priv->stats.tx_window_errors++;
		}
	}

	if (errors) {
		if (netif_msg_tx_err(priv)) {
			bba_printk(KERN_DEBUG, "tx errors, status %8.8x.\n",
				   status);
		}
	}
	return errors;
}

/**
 *
 */
#if 0
static void bba_weird(u8 status, struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;

	/* XXX priv->stats.rx_missed_errors += 0x06-0x07; */
	/* XXX write 0 to 0x06-0x07 */

	if ((status & BBA_RX_STATUS_RERR) || (status & BBA_RX_STATUS_FAE)) {
		bba_rx_err(status, dev);
	}

}

/**
 *
 */
static void bba_tx_timeout(struct net_device *dev)
{
}
#endif

/**
 *
 */
static void bba_interrupt(struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	u8 ir, imr, status;

	ir = bba_in8(priv,BBA_IR);
	imr = bba_in8(priv,BBA_IMR);
	status = ir & imr;

	/* close possible races with dev_close */
	if (unlikely(!netif_running(dev))) {
		bba_out8(priv,BBA_IR, status);
		bba_out8(priv,BBA_IMR, 0x00);
		return;
        }

	while (status) {
		bba_out8(priv,BBA_IR, status);

		if (status & BBA_IR_RBFI) {
			bba_rx(dev, 0x0f);
		}
		if (status & BBA_IR_RI) {
			bba_rx(dev, 0x0f);
		}
		if (status & BBA_IR_REI) {
			bba_rx_err(bba_in8(priv,BBA_LRPS), dev);
		}
		if (status & BBA_IR_TI) {
			bba_tx_err(bba_in8(priv,BBA_LTPS), dev);
			/* allow more packets to be sent */
			netif_wake_queue(dev);
		}
		if (status & BBA_IR_TEI) {
			bba_tx_err(bba_in8(priv,BBA_LTPS), dev);
			/* allow more packets to be sent */
			netif_wake_queue(dev);
		}
		if (status & BBA_IR_FIFOEI) {
			/* allow more packets to be sent */
			netif_wake_queue(dev);
		}
		if (status & BBA_IR_BUSEI) {
		}
		if (status & BBA_IR_FRAGI) {
		}

		ir = bba_in8(priv,BBA_IR);
		imr = bba_in8(priv,BBA_IMR);
		status = ir & imr;
	}
}

/**
 *
 */
static int bba_reset(struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	/* unknown, mx register 0x60 */
	bba_out8(priv,0x60, 0);
	udelay(1000);

	/* unknown, command register 0x0f */
	bba_cmd_in8_slow(priv,0x0f);
	udelay(1000);
	
	/* software reset (write 1 then write 0) */
	bba_out8(priv,BBA_NCRA, BBA_NCRA_RESET);
	udelay(100);
	bba_out8(priv,BBA_NCRA, 0);

	/* unknown, command register 0x01 */
	/* XXX obtain bits needed for challenge/response calculation later */
	priv->revid = bba_cmd_in8(priv,0x01);

	/* unknown, command registers 0x04, 0x05 */
	bba_cmd_outs(priv,0x04, priv->__0x04_init, 2);
	bba_cmd_out8(priv,0x05, priv->__0x05_init);

	/*
	 * These initializations seem to limit the final port speed to 10Mbps
	 * half duplex. Bypassing them, allows one to set other port speeds.
	 * But, remember that the bba spi-like bus clock operates at 32MHz.
	 * ---Albert Herranz
	 */
	
	/* unknown, mx registers 0x5b, 0x5c, 0x5e */
	bba_out8(priv,0x5b, bba_in8(priv,0x5b) & ~(1 << 7));
	bba_out8(priv,0x5e, 1); /* without this the BBA goes at half the speed */
	bba_out8(priv,0x5c, bba_in8(priv,0x5c) | 4);
	udelay(1000);
	
	/* accept broadcast, assert int for every two packets received */
	bba_out8(priv,BBA_NCRB, BBA_NCRB_AB | BBA_NCRB_2_PACKETS_PER_INT);
	
	/* setup receive interrupt time out, in 40ns units */
	bba_out8(priv,BBA_RXINTT, 0x00);
	bba_out8(priv,BBA_RXINTT+1, 0x06); /* 0x0600 = 61us */
	
	/* auto RX full recovery */
	bba_out8(priv,BBA_MISC2, BBA_MISC2_AUTORCVR);
	
	/* initialize packet memory layout */
	bba_out12(priv,BBA_TLBP, BBA_INIT_TLBP);
	bba_out12(priv,BBA_BP, BBA_INIT_BP);
	bba_out12(priv,BBA_RHBP, BBA_INIT_RHBP);
	
	/* set receive page pointers */
	bba_out12(priv,BBA_RWP, BBA_INIT_RWP);
	bba_out12(priv,BBA_RRP, BBA_INIT_RRP);
	
	/* packet memory won't contain packets with RW, FO, CRC errors */
	bba_out8(priv,BBA_GCA, BBA_GCA_ARXERRB);
	
	/* retrieve MAC address */
	bba_ins(priv,BBA_NAFR_PAR0,dev->dev_addr, ETH_ALEN);
	if (!is_valid_ether_addr(dev->dev_addr)) {
		random_ether_addr(dev->dev_addr);
	}
	
	/* setup broadcast address */
	memset(dev->broadcast, 0xFF, ETH_ALEN);
	
	/* clear all interrupts */
	bba_out8(priv,BBA_IR, 0xFF);
	
	/* enable all interrupts */
	bba_out8(priv,BBA_IMR, 0xFF & ~BBA_IMR_FIFOEIM);
	
	/* unknown, short command registers 0x02 */
	/* XXX enable interrupts on the EXI glue logic */
	bba_cmd_out8(priv,0x02, BBA_CMD_IR_MASKNONE);

	/* start receiver */
	bba_out8(priv,BBA_NCRA, BBA_NCRA_SR);
	
	
	/* DO NOT clear interrupts on the EXI glue logic !!! */
	/* we need that initial interrupts for the challenge/response */
	return 0;		/* OK */
}

/**
 *
 */
unsigned long bba_calc_response(unsigned long val, struct bba_private *priv)
{
	u8 revid_0, revid_eth_0, revid_eth_1;
	revid_0 = priv->revid;
	revid_eth_0 = priv->__0x04_init[0];
	revid_eth_1 = priv->__0x04_init[1];

	u8 i0, i1, i2, i3;
	i0 = (val & 0xff000000) >> 24;
	i1 = (val & 0x00ff0000) >> 16;
	i2 = (val & 0x0000ff00) >> 8;
	i3 = (val & 0x000000ff);
	
	u8 c0, c1, c2, c3;
	c0 = ((i0 + i1 * 0xc1 + 0x18 + revid_0) ^ (i3 * i2 + 0x90)) & 0xff;
	c1 = ((i1 + i2 + 0x90) ^ (c0 + i0 - 0xc1)) & 0xff;
	c2 = ((i2 + 0xc8) ^ (c0 + ((revid_eth_0 + revid_0 * 0x23) ^ 0x19))) 
		& 0xff;
	c3 = ((i0 + 0xc1) ^ (i3 + ((revid_eth_1 + 0xc8) ^ 0x90))) & 0xff;
	
	return ((c0 << 24) | (c1 << 16) | (c2 << 8) | c3);
}

static int bba_event_handler(int channel,void *dev0)
{
	struct net_device *dev = (struct net_device*)dev0;
	struct bba_private *priv = (struct bba_private*)dev->priv;
	/* delay interrupt processing to the tasklet.
	   we can't use the regular processing because it relies on the bba_
	   functions which sleep.  We can't sleep from an IRQ handler.
        */
	
	/* schedule the kthread */
	atomic_inc(&priv->num_interrupts);
	wake_up(&priv->wait_queue);
	return 0;
}

static void bba_handle_interrupt(struct bba_private *priv)
{
	register u8 status, mask;

	/* get interrupt status from EXI glue */
	status = bba_cmd_in8(priv,0x03);

	/* XXX mask all EXI glue interrupts */
	bba_cmd_out8(priv,0x02, BBA_CMD_IR_MASKALL);

	/* start with the usual case */
	mask = (1<<7);

	/* normal interrupt from the macronix chip */
	if (status & mask) {
		/* call our interrupt handler */
		bba_interrupt(priv->dev);
		goto out;
	}

	/* "killing" interrupt, try to not get one of these! */
	mask >>= 1;
	if (status & mask) {
		DPRINTK("bba: killing interrupt!\n");
		/* reset the adapter so that we can continue working */
		bba_reset(priv->dev);
		goto out;
	}
	
	/* command error interrupt, haven't seen one yet */
	mask >>= 1;
	if (status & mask) {
		goto out;
	}
	
	/* challenge/response interrupt */
	mask >>= 1;
	if (status & mask) {
		unsigned long response;
		unsigned long challenge;
		
		/* kids, don't do it without an adult present */
		bba_cmd_out8(priv,0x05, priv->__0x05_init);
		bba_cmd_ins(priv,0x08, &challenge, sizeof(challenge));
		response = bba_calc_response(challenge, priv);
		bba_cmd_outs(priv,0x09, &response, sizeof(response));

		goto out;
	}

	/* challenge/response status interrupt */
	mask >>= 1;
	if (status & mask) {
		/* better get a "1" here ... */
		u8 result = bba_cmd_in8(priv,0x0b);
		if (result != 1) {
			bba_printk(KERN_DEBUG,
				   "challenge failed! (result=%d)\n", result);
		}
		goto out;
	}

	/* should not happen, treat as normal interrupt in any case */
	DPRINTK("bba: unknown interrupt type = %d\n", status);

out:
	/* assert interrupt */
	bba_cmd_out8(priv,0x03, mask);

	/* enable interrupts again */
	bba_cmd_out8(priv,0x02, BBA_CMD_IR_MASKNONE);
}

static int int_kthread(void *param)
{
	unsigned long flags;
	struct bba_private *priv = (struct bba_private *)param;
	/* set my priority through the roof */
	daemonize("knetexi");
	current->flags |= PF_NOFREEZE;
        /* this next section is copied from __setscheduler, it's static so
	   we can't use it here */
	/*local_irq_save(flags);
        current->policy = SCHED_FIFO;
        current->rt_priority = MAX_RT_PRIO - 1;
	current->prio = MAX_USER_RT_PRIO - 1 - current->rt_priority;
	local_irq_restore(flags); */
	/* go into running state */
	__set_current_state(TASK_RUNNING);
	do {
		/* wait for a wakeup */
		wait_event(priv->wait_queue,
			   atomic_read(&priv->num_interrupts) || priv->skb);
		/* process the interrupt */
		if (atomic_read(&priv->num_interrupts)) {
			atomic_dec(&priv->num_interrupts);
			bba_handle_interrupt(priv);
		}
		if (priv->skb) {
			/* send the packet */
			bba_do_xmit(priv);
		}
		
	} while (!kthread_should_stop());
	
	return 0;
}
/**
 *
 */
static int bba_init_one(struct exi_device *exi_dev)
{
	struct net_device *dev = NULL;
	struct bba_private *priv;
	int err;

	dev = alloc_etherdev(sizeof(struct bba_private));
	if (!dev) {
		printk(KERN_ERR "unable to allocate net device\n");
		err = -ENOMEM;
		goto err_out;
	}

	/* no IRQ, we use the event system from the EXI driver */
	dev->irq = 0;

	dev->open = bba_open;
	dev->stop = bba_close;
	dev->hard_start_xmit = bba_start_xmit;
	dev->get_stats = bba_get_stats;
	/* XXX dev->tx_timeout = bba_tx_timeout; */

	SET_MODULE_OWNER(dev);
	SET_NETDEV_DEV(dev,&exi_dev->dev);

	priv = netdev_priv(dev);
	priv->dev = dev;
	priv->exi_dev = exi_dev;
	priv->skb = NULL;
	//spin_lock_init(&priv->lock);

	priv->revid = 0xf0;
	priv->__0x04_init[0] = 0xd1;
	priv->__0x04_init[1] = 0x07;
	priv->__0x05_init = 0x4e;
	priv->dma_send_ptr = align(priv->dma_send_buffer);
	priv->dma_recv_ptr = align(priv->dma_recv_buffer);
	atomic_set(&priv->num_interrupts,0);
	init_waitqueue_head(&priv->wait_queue);
	priv->interrupt_thread = kthread_run(int_kthread,priv,"knetexi");
	
	ether_setup(dev);
	dev->flags &= ~IFF_MULTICAST;

	bba_reset(dev);
	
	err = register_netdev(dev);
	if (err) {
		printk(KERN_ERR PFX "Cannot register net device, aborting\n");
		goto err_out_free_dev;
	}
	
	exi_set_driver_data(exi_dev,dev);
	return 0;

err_out_free_dev:
	free_netdev(dev);

err_out:
	return err;
}

static void bba_remove(struct exi_device *dev)
{
	struct net_device *bba_dev = (struct net_device*)exi_get_driver_data(dev);
	struct bba_private *priv = (struct bba_private*)bba_dev->priv;
	
	if (bba_dev) {
		kthread_stop(priv->interrupt_thread);
		
		unregister_netdev(bba_dev);
		free_netdev(bba_dev);

		exi_set_driver_data(dev,NULL);
	}
}
/**
 *
 */
static int bba_probe(struct exi_device *dev)
{
	int ret = 0;
	long exi_id;

	exi_id = bba_exi_probe(dev);
	if (exi_id != BBA_EXI_ID) {
		bba_printk(KERN_WARNING, "Nintendo GameCube Broadband Adapter"
			   " not found\n");
		return -ENODEV;
	}
	
	ret = bba_init_one(dev);
	
	return ret;
}

static struct exi_driver bba_exi_driver = {
	.name = "Nintendo Gamecube Broadband Adapter",
	.eid = {
		.channel = BBA_EXI_CHANNEL,
		.device  = BBA_EXI_DEVICE,
		.id      = BBA_EXI_ID
	},
	.frequency = BBA_EXI_FREQ,
	.probe = bba_probe,
	.remove = bba_remove 
};
/**
 *	bba_init_module -  driver initialization routine
 *
 *
 */
static int __init bba_init_module(void)
{
	return exi_register_driver(&bba_exi_driver);
}

/**
 *
 */
static void __exit bba_exit_module(void)
{
	exi_unregister_driver(&bba_exi_driver);
}

module_init(bba_init_module);
module_exit(bba_exit_module);

