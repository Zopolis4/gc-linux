/**
 * gamecube_bba.c - Broadband Adapter driver for Nintendo's GameCube
 * Copyright(C) 2004 Albert Herranz
 *
 * Based on previous work by Stefan Esser, Franz Lehner, Costis and tmbinc.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2. See the file COPYING for more details.
 */

#define DEBUG

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
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <asm/system.h>
#include <asm/io.h>

#define BBA_DEBUG (defined(DEBUG))

#ifdef BBA_DEBUG
#  define DPRINTK(fmt, args...) \
          printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

#define MISSING_EXI_FRAMEWORK 1
#include <linux/exi.h>


/* ----8<--------8<--------8<--------8<--------8<--------8<--------8<---- */

/* XXX The following stuff should be part of the exi-driver when finished.    */
/* XXX ---Albert Herranz                                                      */

/* XXX hack to avoid kernel crash while we lack a proper initialized device */
#undef dev_printk
#define dev_printk(level, dev, format, arg...)  \
        printk(level "exi bus0: " format , ## arg)

#define EXI_MAX_CHANNELS  3	/* channels on the EXI bus */
#define EXI_MAX_EVENTS    3	/* types of events on the EXI bus */

#define EXI_EVENT_IRQ     0
#define EXI_EVENT_INSERT  1
#define EXI_EVENT_TC      2

#define EXI_READ  0
#define EXI_WRITE 1

#define IRQ_EXI 4

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

struct exi_bus_dev_private {
	spinlock_t lock;

	spinlock_t select_lock;
	unsigned long select_flags;

	struct exi_dev_event events[EXI_MAX_CHANNELS][EXI_MAX_EVENTS];
};

//#define exi_bus_dev_priv(bus) ((bus)->priv)
static struct exi_bus_dev_private __private;
#define exi_bus_dev_priv(bus) (&__private)

// these functions are used by gc_memcard.c ...
void exi_select(int channel, int device, int freq);
void exi_deselect(int channel);
void exi_imm(int channel, void *data, int len, int mode, int zero);
void exi_sync(int channel);

static void exi_imm_ex(int channel, void *data, int len, int mode);

static irqreturn_t exi_irq_handler(int irq, void *dev_id, struct pt_regs *regs);

int exi_register_event(int channel, int event_id,
		       int (*handler) (int, int, void *), void *dev);
int exi_unregister_event(int channel, int event_id);

static inline int exi_trigger_event(int channel, int event_id);

static int exi_init(void);
static void exi_exit(void);

/* exi_select: enable chip select, set speed */

static int selected = 0;
void exi_select(int channel, int device, int freq)
{
	struct exi_bus_dev_private *priv = exi_bus_dev_priv(&exi_bus_dev);
	spin_lock_irqsave(priv->select_lock, priv->select_flags);

	volatile unsigned long *exi = (volatile unsigned long *)0xCC006800;
	selected++;
	if (selected != 1)
		panic("-------- select while selected!\n");
	long d;
	// exi_select
	d = exi[channel * 5];
	d &= 0x405;
	d |= ((1 << device) << 7) | (freq << 4);
	exi[channel * 5] = d;
}

/* disable chipselect */
void exi_deselect(int channel)
{
	volatile unsigned long *exi = (volatile unsigned long *)0xCC006800;
	selected--;
	if (selected)
		panic("deselect broken!");
	exi[channel * 5] &= 0x405;
	struct exi_bus_dev_private *priv = exi_bus_dev_priv(&exi_bus_dev);
	spin_unlock_irqrestore(priv->select_lock, priv->select_flags);
}

/* dirty way for asynchronous reads */
static void *exi_last_addr;
static int exi_last_len;

/* mode?Read:Write len bytes to/from channel */
/* when read, data will be written back in exi_sync */
void exi_imm(int channel, void *data, int len, int mode, int zero)
{
	volatile unsigned long *exi = (volatile unsigned long *)0xCC006800;
	if (mode == EXI_WRITE)
		exi[channel * 5 + 4] = *(unsigned long *)data;
	exi[channel * 5 + 3] = ((len - 1) << 4) | (mode << 2) | 1;
	if (mode == EXI_READ) {
		exi_last_addr = data;
		exi_last_len = len;
	} else {
		exi_last_addr = 0;
		exi_last_len = 0;
	}
}

/* Wait until transfer is done, write back data */
void exi_sync(int channel)
{
	volatile unsigned long *exi = (volatile unsigned long *)0xCC006800;
	while (exi[channel * 5 + 3] & 1) ;

	if (exi_last_addr) {
		int i;
		unsigned long d;
		d = exi[channel * 5 + 4];
		for (i = 0; i < exi_last_len; ++i)
			((unsigned char *)exi_last_addr)[i] =
			    (d >> ((3 - i) * 8)) & 0xFF;
	}
}

/* simple wrapper for transfers > 4bytes */
static void exi_imm_ex(int channel, void *data, int len, int mode)
{
	unsigned char *d = (unsigned char *)data;
	while (len) {
		int tc = len;
		if (tc > 4)
			tc = 4;
		exi_imm(channel, d, tc, mode, 0);
		exi_sync(channel);
		len -= tc;
		d += tc;
	}
}

/**
 *	exi_irq_handler - handle interrupts from EXI devices
 *	@irq: interrupt line
 *	@dev_id: device from where the interrupt comes (exi_bus_dev)
 *	@regs: processor register set
 *
 *	Handles External Interface interrupts from EXI devices.
 *
 *	Usually, the exi_bus_dev handles all interrupts and passes them
 *	to EXI devices that have asked previously for specific subtypes
 *	of these interrupts.
 */
static irqreturn_t exi_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	struct exi_bus_dev_private *priv = exi_bus_dev_priv(&exi_bus_dev);

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

		writel(csr | status, reg);	/* ack all, XXX really? */
		//int devnum = (csr & EXI_CSR_CSMASK) >> 7;

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
static inline int exi_trigger_event(int channel, int event_id)
{
	struct exi_bus_dev_private *priv = exi_bus_dev_priv(&exi_bus_dev);
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
	struct exi_bus_dev_private *priv = exi_bus_dev_priv(&exi_bus_dev);
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
	struct exi_bus_dev_private *priv = exi_bus_dev_priv(&exi_bus_dev);
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
 *
 */
static int exi_init()
{
	struct exi_bus_dev_private *priv = exi_bus_dev_priv(&exi_bus_dev);
	int err = 0;

	spin_lock_init(priv->lock);
	spin_lock_init(priv->select_lock);

	err = request_irq(IRQ_EXI, exi_irq_handler, SA_SHIRQ, "exi", NULL);
	if (err) {
		dev_dbg(&exi_bus_dev, "unable to get IRQ %d\n", IRQ_EXI);
	}

	return err;
}

/**
 *
 */
static void exi_exit(void)
{
	free_irq(IRQ_EXI, NULL);
}

/* ----8<--------8<--------8<--------8<--------8<--------8<--------8<---- */


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

#define BBA_EXI_CHANNEL 0
#define BBA_EXI_DEVICE  2
#define BBA_EXI_FREQ    5

#define BBA_CMD_IR_MASKALL  0x00
#define BBA_CMD_IR_MASKNONE 0xf8

static void bba_cmd_ins(int reg, void *val, int len);
static void bba_cmd_outs(int reg, void *val, int len);
static void bba_ins(int reg, void *val, int len);
static void bba_outs(int reg, void *val, int len);

#define bba_select()   exi_select(BBA_EXI_CHANNEL, BBA_EXI_DEVICE, BBA_EXI_FREQ)
#define bba_deselect() exi_deselect(BBA_EXI_CHANNEL)

static inline void bba_cmd_ins_nosel(int reg, void *val, int len)
{
	u16 req;
	req = reg << 8;
	exi_imm(BBA_EXI_CHANNEL, &req, sizeof(req), EXI_WRITE, 0);
	exi_sync(BBA_EXI_CHANNEL);
	exi_imm_ex(BBA_EXI_CHANNEL, val, len, EXI_READ);
}

static void bba_cmd_ins(int reg, void *val, int len)
{
	bba_select();
	bba_cmd_ins_nosel(reg, val, len);
	bba_deselect();
}

static inline void bba_cmd_outs_nosel(int reg, void *val, int len)
{
	u16 req;
	req = (reg << 8) | 0x4000;
	exi_imm(BBA_EXI_CHANNEL, &req, sizeof(req), EXI_WRITE, 0);
	exi_sync(BBA_EXI_CHANNEL);
	exi_imm_ex(BBA_EXI_CHANNEL, val, len, EXI_WRITE);
}

static void bba_cmd_outs(int reg, void *val, int len)
{
	bba_select();
	bba_cmd_outs_nosel(reg, val, len);
	bba_deselect();
}

static inline u8 bba_cmd_in8(int reg)
{
	u8 val;
	bba_cmd_ins(reg, &val, sizeof(val));
	return val;
}

static u8 bba_cmd_in8_slow(int reg)
{
	u8 val;
	bba_select();
	bba_cmd_ins_nosel(reg, &val, sizeof(val));
	udelay(200);
	bba_deselect();
	return val;
}

static inline void bba_cmd_out8(int reg, u8 val)
{
	bba_cmd_outs(reg, &val, sizeof(val));
}

static inline u8 bba_in8(int reg)
{
	u8 val;
	bba_ins(reg, &val, sizeof(val));
	return val;
}

static inline void bba_out8(int reg, u8 val)
{
	bba_outs(reg, &val, sizeof(val));
}

#define bba_in12(reg)     ((bba_in8(reg)&0xff)|((bba_in8((reg)+1)&0x0f)<<8))
#define bba_out12(reg,val) do { bba_out8((reg),(val)&0xff); \
	                        bba_out8((reg)+1,((val)&0x0f00)>>8); } while(0)

static inline void bba_ins_nosel(int reg, void *val, int len)
{
	u32 req;
	req = (reg << 8) | 0x80000000;
	exi_imm(BBA_EXI_CHANNEL, &req, sizeof(req), EXI_WRITE, 0);
	exi_sync(BBA_EXI_CHANNEL);
	exi_imm_ex(BBA_EXI_CHANNEL, val, len, EXI_READ);
}

static void bba_ins(int reg, void *val, int len)
{
	bba_select();
	bba_ins_nosel(reg, val, len);
	bba_deselect();
}

static inline void bba_outs_nosel(int reg, void *val, int len)
{
	u32 req;
	req = (reg << 8) | 0xC0000000;
	exi_imm(BBA_EXI_CHANNEL, &req, sizeof(req), EXI_WRITE, 0);
	exi_sync(BBA_EXI_CHANNEL);
	exi_imm_ex(BBA_EXI_CHANNEL, val, len, EXI_WRITE);
}

static void bba_outs(int reg, void *val, int len)
{
	bba_select();
	bba_outs_nosel(reg, val, len);
	bba_deselect();
}

/**
 * Nintendo GameCube Broadband Adapter driver.
 *
 */

#define DRV_MODULE_NAME   "gc-net"
#define DRV_DESCRIPTION   "Nintendo GameCube Broadband Adapter"
#define DRV_AUTHOR        "Albert Herranz"

char bba_driver_name[] = DRV_MODULE_NAME;
char bba_driver_string[] = DRV_DESCRIPTION;
char bba_driver_version[] = "0.1-isobel";
char bba_copyright[] = "Copyright (C) 2004 " DRV_AUTHOR;

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_LICENSE(GPL);

#define PFX               DRV_MODULE_NAME ": "

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


struct bba_private {
	spinlock_t lock;

	u32 msg_enable;
	u8 revid;
	u8 __0x04_init[2];
	u8 __0x05_init;

	struct net_device *dev;
	struct net_device_stats stats;
};

static int bba_open(struct net_device *dev);
static int bba_close(struct net_device *dev);
static struct net_device_stats *bba_get_stats(struct net_device *dev);

static int __devinit bba_probe(void);
static int __devinit bba_init_one(void);
static int __init bba_init_module(void);
static void __exit bba_exit_module(void);
int bba_event_handler(int channel, int event, void *dev0);

static int bba_reset(struct net_device *dev);

/**
 *
 */
static int bba_open(struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	unsigned long flags;
	int ret;

	/* XXX seems that interrupts come from EXI Channel 2, but driver
	 * XXX operation is accomplished through EXI Channel 0 ...
	 */
	ret = exi_register_event(2, EXI_EVENT_IRQ, bba_event_handler, dev);
	if (ret < 0) {
		dev_dbg(&priv->edev->dev, "unable to register EXI event %d\n",
			EXI_EVENT_IRQ);
		return ret;
	}

	spin_lock_irqsave(&priv->lock, flags);
	ret = bba_reset(dev);
	spin_unlock_irqrestore(&priv->lock, flags);

	netif_start_queue(dev);

	return ret;
}

/**
 *
 */
static int bba_close(struct net_device *dev)
{
	exi_unregister_event(2, EXI_EVENT_IRQ);
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

/**
 *
 */
static int bba_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	unsigned long flags;

	/* we are not able to send packets greater than this */
	if (skb->len > BBA_TX_MAX_PACKET_SIZE) {
		dev_kfree_skb(skb);
		priv->stats.tx_dropped++;
		return 0;
	}

	/* we need to make sure next section will run without disruptions */
	spin_lock_irqsave(&priv->lock, flags);

	/* if the TXFIFO is in use we'll try it later when free */
	if (bba_in8(BBA_NCRA) & (BBA_NCRA_ST0 | BBA_NCRA_ST1)) {
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&priv->lock, flags);
		return 1;
	}

	/* tell the card about the length of this packet */
	bba_out12(BBA_TXFIFOCNT, skb->len);

	/* store the packet in the TXFIFO, including padding if needed */
	bba_select();
	bba_outs_nosel(BBA_WRTXFIFOD, skb->data, skb->len);
	if (skb->len < ETH_ZLEN) {
		u8 pad[ETH_ZLEN];
		int pad_len = ETH_ZLEN - skb->len;
		memset(pad, 0, pad_len);
		bba_outs_nosel(BBA_WRTXFIFOD, pad, pad_len);
	}
	bba_deselect();

	/* tell the card to send the packet right now */
	bba_out8(BBA_NCRA, (bba_in8(BBA_NCRA) | BBA_NCRA_ST1) & ~BBA_NCRA_ST0);

	/* stop the queue as we can only send one packet each time */
	netif_stop_queue(dev);

	/* end of critical section */
	spin_unlock_irqrestore(&priv->lock, flags);

	dev->trans_start = jiffies;
	priv->stats.tx_bytes += skb->len;
	priv->stats.tx_packets++;

	dev_kfree_skb(skb);

	return 0;
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
			dev_dbg(&priv->edev->dev,
				"rx errors, status %8.8x.\n", status);
		}

		/* stop receiver */
		bba_out8(BBA_NCRA, bba_in8(BBA_NCRA) & ~BBA_NCRA_SR);

		/* initialize page pointers */
		bba_out12(BBA_RWP, BBA_INIT_RWP);
		bba_out12(BBA_RRP, BBA_INIT_RRP);

		/* start receiver again */
		bba_out8(BBA_NCRA, bba_in8(BBA_NCRA) | BBA_NCRA_SR);
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

	rwp = bba_in12(BBA_RWP);
	rrp = bba_in12(BBA_RRP);

	while (netif_running(dev) && received < budget && rrp != rwp) {
		bba_ins(rrp << 8, &descr, sizeof(descr));
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
		skb = dev_alloc_skb(size + 2);
		if (!skb) {
			priv->stats.rx_dropped++;
			continue;
		}

		skb->dev = dev;
		skb_reserve(skb, 2);	/* align */
		skb_put(skb, size);

		pos = (rrp << 8) + 4;	/* skip descriptor */
		top = (BBA_INIT_RHBP + 1) << 8;

		if ((pos + size) < top) {
			/* full packet in one chunk */
			bba_ins(pos, skb->data, size);
		} else {
			/* packet wrapped */
			int chunk_size = top - pos;

			bba_ins(pos, skb->data, chunk_size);
			rrp = BBA_INIT_RRP;
			bba_ins(rrp << 8, skb->data + chunk_size,
				size - chunk_size);
		}

		skb->protocol = eth_type_trans(skb, dev);

		dev->last_rx = jiffies;
		priv->stats.rx_bytes += size;
		priv->stats.rx_packets++;

		netif_rx(skb);
		received++;

		bba_out12(BBA_RRP, descr.next_packet_ptr);

		rwp = bba_in12(BBA_RWP);
		rrp = bba_in12(BBA_RRP);
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
			dev_dbg(&priv->edev->dev, "tx errors, status %8.8x.\n",
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

	//priv->stats.rx_missed_errors += 0x06-0x07;
	// write 0 to 0x06-0x07

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
static void inline bba_interrupt(struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;
	unsigned long flags;
	u8 ir, imr, status;
	int loops = 0;

	ir = bba_in8(BBA_IR);
	imr = bba_in8(BBA_IMR);
	status = ir & imr;

	while (status) {
		bba_out8(BBA_IR, status);

		if (status & BBA_IR_RBFI) {
			spin_lock_irqsave(&priv->lock, flags);
			bba_rx(dev, 0x0f);
			spin_unlock_irqrestore(&priv->lock, flags);
		}
		if (status & BBA_IR_RI) {
			spin_lock_irqsave(&priv->lock, flags);
			bba_rx(dev, 0x0f);
			spin_unlock_irqrestore(&priv->lock, flags);
		}
		if (status & BBA_IR_REI) {
			spin_lock_irqsave(&priv->lock, flags);
			bba_rx_err(bba_in8(BBA_LRPS), dev);
			spin_unlock_irqrestore(&priv->lock, flags);
		}
		if (status & BBA_IR_TI) {
			bba_tx_err(bba_in8(BBA_LTPS), dev);
			/* allow more packets to be sent */
			netif_wake_queue(dev);
		}
		if (status & BBA_IR_TEI) {
			bba_tx_err(bba_in8(BBA_LTPS), dev);
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

		ir = bba_in8(BBA_IR);
		imr = bba_in8(BBA_IMR);
		status = ir & imr;
		loops++;
	}

	if (loops > 10)
		DPRINTK("a lot of interrupt work (%d loops)\n", loops);

	/* wake up xmit queue in case transmitter is idle */
	if ((bba_in8(BBA_NCRA) & (BBA_NCRA_ST0 | BBA_NCRA_ST1)) == 0) {
		netif_wake_queue(dev);
	}
}

/**
 *
 */
static int bba_reset(struct net_device *dev)
{
	struct bba_private *priv = (struct bba_private *)dev->priv;

	/* unknown, mx register 0x60 */
	bba_out8(0x60, 0);
	udelay(1000);

	/* unknown, command register 0x0f */
	bba_cmd_in8_slow(0x0f);
	udelay(1000);

	/* software reset (write 1 then write 0) */
	bba_out8(BBA_NCRA, BBA_NCRA_RESET);
	udelay(100);
	bba_out8(BBA_NCRA, 0);

	/* unknown, command register 0x01 */
	/* XXX obtain bits needed for challenge/response calculation later */
	priv->revid = bba_cmd_in8(0x01);

	/* unknown, command registers 0x04, 0x05 */
	bba_cmd_outs(0x04, priv->__0x04_init, 2);
	bba_cmd_out8(0x05, priv->__0x05_init);

	/* unknown, mx registers 0x5b, 0x5c, 0x5e */
	bba_out8(0x5b, bba_in8(0x5b) & ~(1 << 7));
	bba_out8(0x5e, 1);
	bba_out8(0x5c, bba_in8(0x5c) | 4);

	udelay(1000);

	/* accept broadcast, assert int for every packet received */
	bba_out8(BBA_NCRB, BBA_NCRB_AB | BBA_NCRB_2_PACKETS_PER_INT);

	/* setup receive interrupt time out, in 40ns units */
	bba_out8(BBA_RXINTT, 0x00);
	bba_out8(BBA_RXINTT+1, 0x06); /* 0x0600 = 61us */

	/* auto RX full recovery */
	bba_out8(BBA_MISC2, BBA_MISC2_AUTORCVR);

	/* initialize packet memory layout */
	bba_out12(BBA_TLBP, BBA_INIT_TLBP);
	bba_out12(BBA_BP, BBA_INIT_BP);
	bba_out12(BBA_RHBP, BBA_INIT_RHBP);

	/* set receive page pointers */
	bba_out12(BBA_RWP, BBA_INIT_RWP);
	bba_out12(BBA_RRP, BBA_INIT_RRP);

	/* start receiver */
	bba_out8(BBA_NCRA, BBA_NCRA_SR);

	/* packet memory won't contain packets with RW, FO, CRC errors */
	bba_out8(BBA_GCA, BBA_GCA_ARXERRB);

	/* retrieve MAC address */
	bba_ins(BBA_NAFR_PAR0, dev->dev_addr, ETH_ALEN);
	if (!is_valid_ether_addr(dev->dev_addr)) {
		random_ether_addr(dev->dev_addr);
	}

	/* setup broadcast address */
	memset(dev->broadcast, 0xff, ETH_ALEN);

	/* clear all interrupts */
	bba_out8(BBA_IR, 0xFF);	// clear all irqs

	/* enable all interrupts */
	bba_out8(BBA_IMR, 0xFF & ~BBA_IMR_FIFOEIM);

	/* unknown, short command registers 0x02 */
	/* XXX enable interrupts on the EXI glue logic */
	bba_cmd_out8(0x2, BBA_CMD_IR_MASKNONE);

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
	c0 = ((i0 + i1 * 0xc1 + 0x18 + revid_0) ^ (i3 * i2 + 0x90)
	    ) & 0xff;
	c1 = ((i1 + i2 + 0x90) ^ (c0 + i0 - 0xc1)
	    ) & 0xff;
	c2 = ((i2 + 0xc8) ^ (c0 + ((revid_eth_0 + revid_0 * 0x23) ^ 0x19))
	    ) & 0xff;
	c3 = ((i0 + 0xc1) ^ (i3 + ((revid_eth_1 + 0xc8) ^ 0x90))
	    ) & 0xff;

	return ((c0 << 24) | (c1 << 16) | (c2 << 8) | c3);
}

/**
 *
 */
int bba_event_handler(int channel, int event, void *dev0)
{
	struct net_device *dev = (struct net_device *)dev0;
	struct bba_private *priv = (struct bba_private *)dev->priv;
	unsigned long flags;
	u8 status;

	/* get interrupt status from EXI glue */
	status = bba_cmd_in8(0x03);

	/* XXX mask all EXI glue interrupts */
	bba_cmd_out8(0x02, BBA_CMD_IR_MASKALL);

	/* normal interrupt from the macronix chip */
	if (status & 0x80) {
		/* assert interrupt */
		bba_cmd_out8(0x03, 0x80);
		/* call our interrupt handler */
		bba_interrupt(dev);
		/* enable interrupts again */
		bba_cmd_out8(0x02, BBA_CMD_IR_MASKNONE);
		return 1;
	}

	/* "killing" interrupt, try to not get one of these! */
	if (status & 0x40) {
		/* assert interrupt, although in this case doesn't help */
		bba_cmd_out8(0x03, 0x40);
		/* reset the adapter so that we can continue working */
		spin_lock_irqsave(&priv->lock, flags);
		bba_reset(dev);
		spin_unlock_irqrestore(&priv->lock, flags);
		/* enable interrupts again */
		bba_cmd_out8(0x02, BBA_CMD_IR_MASKNONE);
		return 1;
	}

	/* command error interrupt, haven't seen one yet */
	if (status & 0x20) {
		/* assert interrupt */
		bba_cmd_out8(0x03, 0x20);
		/* enable interrupts again */
		bba_cmd_out8(0x02, BBA_CMD_IR_MASKNONE);
		return 1;
	}

	/* challenge/response interrupt */
	if (status & 0x10) {
		unsigned long response;
		unsigned long challenge;

		/* kids, don't do it without an adult present */
		bba_cmd_out8(0x05, priv->__0x05_init);
		bba_cmd_ins(0x08, &challenge, sizeof(challenge));
		response = bba_calc_response(challenge, priv);
		bba_cmd_outs(0x09, &response, sizeof(response));

		/* assert interrupt */
		bba_cmd_out8(3, 0x10);
		/* enable interrupts again */
		bba_cmd_out8(2, BBA_CMD_IR_MASKNONE);
		return 1;
	}

	/* challenge/response status interrupt */
	if (status & 0x08) {
		/* better get a "1" here ... */
		u8 result = bba_cmd_in8(0x0b);
		if (result != 1) {
			dev_dbg(&priv->edev->dev,
				"challenge failed! (result=%d)\n", result);
		}

		/* assert interrupt */
		bba_cmd_out8(3, 0x08);
		/* enable interrupts again */
		bba_cmd_out8(2, BBA_CMD_IR_MASKNONE);
		return 1;
	}

	printk("GCIF - EXI - ?? %02x\n", status);

	/* should not happen, treat as normal interrupt in any case */
	bba_interrupt(dev);

	/* enable interrupts again */
	bba_cmd_out8(2, BBA_CMD_IR_MASKNONE);

	return 1;
}

static struct net_device *bba_dev = NULL;

/**
 *
 */
static int __devinit bba_init_one(void)
{
	struct net_device *dev = NULL;
	struct bba_private *priv;
	int err;

	dev = alloc_etherdev(sizeof(*priv));
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
	//dev->tx_timeout = bba_tx_timeout;

	SET_MODULE_OWNER(dev);

	priv = netdev_priv(dev);
	priv->dev = dev;
	spin_lock_init(&priv->lock);

	priv->revid = 0xf0;
	priv->__0x04_init[0] = 0xd1;
	priv->__0x04_init[1] = 0x07;
	priv->__0x05_init = 0x4e;

	ether_setup(dev);
	dev->flags &= ~IFF_MULTICAST;

	bba_reset(dev);

	err = register_netdev(dev);
	if (err) {
		printk(KERN_ERR PFX "Cannot register net device, "
		       "aborting.\n");
		goto err_out_free_dev;
	}

	if (bba_dev)
		free_netdev(bba_dev);
	bba_dev = dev;

	return 0;

      err_out_free_dev:
	free_netdev(dev);

      err_out:
	return err;
}

/**
 *
 */
static int __devinit bba_probe(void)
{
	int ret = 0;

	short cmd = 0;
	long exi_id;

	bba_select();
	exi_imm_ex(0, &cmd, 2, EXI_WRITE);
	exi_imm_ex(0, &exi_id, 4, EXI_READ);
	bba_deselect();
	if (exi_id != BBA_EXI_ID) {
		dev_dbg(&exi_bus_dev, "GameCube Broadband Adapter not found\n");
		return -ENODEV;
	}

	ret = bba_init_one();

	return ret;
}

/**
 *	bba_init_module -  driver initialization routine
 *
 *
 */
static int __init bba_init_module(void)
{
	int ret = 0;

	printk(KERN_INFO "%s - version %s\n",
	       bba_driver_string, bba_driver_version);

	printk(KERN_INFO "%s\n", bba_copyright);

	exi_init();

	ret = bba_probe();

	return ret;
}

/**
 *
 */
static void __exit bba_exit_module(void)
{
	unregister_netdev(bba_dev);
	exi_exit();
}

module_init(bba_init_module);
module_exit(bba_exit_module);
