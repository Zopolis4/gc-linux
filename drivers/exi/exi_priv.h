/*
 * drivers/exi/exi_priv.h
 *
 * Nintendo GameCube EXI driver  
 * Copyright (C) 2004-2005 The GameCube Linux Team
 * Copyright (C) 2004,2005 Todd Jeffreys <todd@voidpointer.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __exi_priv__
#define __exi_priv__

#include <linux/interrupt.h>
#include <linux/list.h>
#include <asm/atomic.h>

/* Flags for exi_command->flags */
#define EXI_CHANNEL_0      (0x00000000)
#define EXI_CHANNEL_1      (0x00010000)
#define EXI_CHANNEL_2      (0x00020000)

#define EXI_DEVICE_0       (0x00100000)
#define EXI_DEVICE_1       (0x00200000)
#define EXI_DEVICE_2       (0x00400000)

#define EXI_FREQUENCY_0    (0x00000000)
#define EXI_FREQUENCY_1    (0x01000000)
#define EXI_FREQUENCY_2    (0x02000000)
#define EXI_FREQUENCY_3    (0x03000000)
#define EXI_FREQUENCY_4    (0x04000000)
#define EXI_FREQUENCY_5    (0x05000000)

#define _EXI_DEBUG 1

#define EXI_IRQ 4

#define EXI_MAX_CHANNELS 3
#define EXI_DEVICES_PER_CHANNEL 3

#define EXI_READ  0
#define EXI_WRITE 1
#define EXI_CSR_BASE      (void* __iomem)0xCC006800 
#define EXI_MAR_BASE      (void* __iomem)0xCC006804
#define EXI_LENGTH_BASE   (void* __iomem)0xCC006808
#define EXI_CR_BASE       (void* __iomem)0xCC00680C
#define EXI_IMM_DATA_BASE (void* __iomem)0xCC006810
#define EXI_CHANNEL_SPACING   0x14
#define   EXI_CSR_EXT         (1<<12)
#define   EXI_CSR_EXTINT      (1<<11)
#define   EXI_CSR_EXTINTMASK  (1<<10)
#define   EXI_CSR_CSMASK      (0x7<<7)
#define     EXI_CSR_CS_0      (0x1<<7)  /* Chip Select 001 */
#define     EXI_CSR_CS_1      (0x2<<7)  /* Chip Select 010 */
#define     EXI_CSR_CS_2      (0x4<<7)  /* Chip Select 100 */
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

#define   EXI_MR_TSTART       (1<<0)
#define   EXI_MR_DMA          (1<<1)
#define   EXI_MR_READ         (0<<2)
#define   EXI_MR_WRITE        (1<<2)
#define   EXI_MR_READ_WRITE   (2<<2)
#define   EXI_MR_TLEN(i)      ((i-1)<<4)

#define EXI_CSR(c)    (EXI_CSR_BASE + (c*EXI_CHANNEL_SPACING))
#define EXI_MAR(c)    (EXI_MAR_BASE + (c*EXI_CHANNEL_SPACING))
#define EXI_LENGTH(c) (EXI_LENGTH_BASE + (c*EXI_CHANNEL_SPACING))
#define EXI_CR(c)     (EXI_CR_BASE + (c*EXI_CHANNEL_SPACING))
#define EXI_IMM(c)    (EXI_IMM_DATA_BASE + (c*EXI_CHANNEL_SPACING))

#define EXI0_CSR EXI_CSR(0)
#define EXI1_CSR EXI_CSR(1)
#define EXI2_CSR EXI_CSR(2)


struct exi_locked_data {
	/* per bus information */
	spinlock_t queue_lock;
	struct list_head queue;
	unsigned int cur_command;
	struct tasklet_struct tasklet;
	enum { EXI_IDLE, EXI_WAITING_FOR_TC } exi_state;
	atomic_t tc_interrupt;
};

struct exi_interrupt_handlers {
	exi_irq_handler func;
	void *param;
};

extern struct exi_interrupt_handlers irq_handlers[EXI_MAX_CHANNELS];
extern struct exi_locked_data exi_data[EXI_MAX_CHANNELS];

void exi_tasklet(unsigned long param);
irqreturn_t exi_bus_irq_handler(int irq,void *dev_id,struct pt_regs *regs);

void exi_bus_insert(unsigned int channel,unsigned int bInsert);
u32 exi_synchronous_id(unsigned int channel,unsigned int device);

#endif
