/*
 * drivers/exi/exi-hw.c
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

#include <linux/types.h>
#include <linux/exi.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <asm/io.h>
#include "exi_priv.h"

static inline void exi_select(struct exi_command_group *cmd)
{
	const u32 channel   = (cmd->flags >> 16) & 3;
	const u32 device    = (cmd->flags >> 20) & 7;
	const u32 frequency = (cmd->flags >> 24) & 7;
	void * __iomem const reg = EXI_CSR(channel);
	u32 val = readl(reg);
	
	val &= (EXI_CSR_EXTINTMASK | EXI_CSR_TCINTMASK | EXI_CSR_EXIINTMASK);
	val |= (device << 7) | (frequency << 4);
	
	writel(val,reg);
}

static inline void exi_deselect(struct exi_command_group *cmd)
{
	u32 channel  = (cmd->flags >> 16) & 3;
	void * __iomem const reg = EXI_CSR(channel);
	u32 val;
	
	if (cmd->flags & EXI_DESELECT_UDELAY) {
		udelay(cmd->deselect_udelay);
	}
	
	val = readl(reg) & (EXI_CSR_EXTINTMASK | EXI_CSR_TCINTMASK |
			    EXI_CSR_EXIINTMASK);
	writel(val,reg);
}

static void exi_immediate_transfer(struct exi_command *cmd,
				   unsigned int channel)
{
	void * __iomem const imm_reg = EXI_IMM(channel);
	void * __iomem const cr_reg  = EXI_CR(channel);
	void * __iomem const csr_reg = EXI_CSR(channel);
	u32 val;
	void *data = cmd->data;
	u32 len   = cmd->len;
	
	/* fast path, 4 bytes at a time */
	while (len >= 4) {
		/* write to register */
		if (cmd->flags & EXI_CMD_WRITE)
			val = *((u32*)data);
		else
			val = 0xFFFFFFFF;
		
		writel(val,imm_reg);
		/* go! */
		val = EXI_MR_TSTART | EXI_MR_TLEN(4) | (cmd->flags & 7);
		writel(val,cr_reg);
		/* wait for completion */
		while (readl(cr_reg) & EXI_MR_TSTART) {	}
		/* clear the TCINT now */
		writel(readl(csr_reg) | EXI_CSR_TCINT,csr_reg);
		/* store result on a read */
		if (!(cmd->flags & EXI_CMD_WRITE)) {
			*((u32*)data) = readl(imm_reg);
		}
		/* change pointer and length */
		data += 4;
		len  -= 4;
	}
	/* if we have 1,2,3 remaining bytes */
	if (len) {
		/* write to register */
		if (cmd->flags & EXI_CMD_WRITE) {
			switch (len) {
			case 1:
				val = *((u8*)data) << 24;
				break;
			case 2:
				val = *((u16*)data) << 16;
				break;
			default:
				/* this is really 3, we read an entire 4 bytes
				   at a time since there is no penalty for 
				   reading the extra byte.  EXI hardware will
				   ignore the extra byte anyways. */
				val = *((u32*)data);
				break;
			}
		}
		else {
			val = 0xFFFFFFFF;
		}
		writel(val,imm_reg);
		/* go */
		val = EXI_MR_TSTART | EXI_MR_TLEN(len) | (cmd->flags & 7);
		writel(val,cr_reg);
		/* wait for completion */
		while (readl(cr_reg) & EXI_MR_TSTART) { }
		/* clear the TCINT now */
		writel(readl(csr_reg) | EXI_CSR_TCINT,csr_reg);
		/* store result on a read */
		if (!(cmd->flags & EXI_CMD_WRITE)) {
			val = readl(imm_reg);
			switch (len) {
			case 1:
				*((u8*)data) = (u8)(val >> 24);
				break;
			case 2:
				*((u16*)data) = (u16)(val >> 16);
				break;
			default:
				/* this is really a 3 byte read */
				*((u16*)data) = (u16)(val >> 16);
				*((u8*)(data+2)) = (u8)(val >> 8);
				break;
			}
		}
	}
}

static void exi_dma_transfer(struct exi_command *subcmd,
				    unsigned int channel)
{
	void * __iomem reg;
	u32 val;
	/* flush the cache when writing */
	if (subcmd->flags & EXI_CMD_WRITE) {
		flush_dcache_range((u32)subcmd->data,
				   (u32)subcmd->data + subcmd->len);
	}
	/* convert to physical */
	val = virt_to_phys(subcmd->data);
	writel(val,EXI_MAR(channel));
	/* write length */
	writel(subcmd->len,EXI_LENGTH(channel));
	/* clear the IMM area */
	writel(0xFFFFFFFF,EXI_IMM(channel));
	/* enable the TC interrupt */
	reg = EXI_CSR(channel);
	val = readl(reg) | EXI_CSR_TCINTMASK | EXI_CSR_TCINT;
	writel(val,reg);
	/* go! */
	val = EXI_MR_TSTART | EXI_MR_DMA | (subcmd->flags & 7);
	writel(val,EXI_CR(channel));
}

static void exi_complete_command(struct exi_locked_data *data,
				 struct exi_command *subcmd)
{
	struct exi_command_group *cmd;
	unsigned long flags;

	cmd = (struct exi_command_group*)data->queue.next;
	/* restore idle flag */
	data->exi_state = EXI_IDLE;
	/* increment the count now */
	data->cur_command++;
	if (data->cur_command >= cmd->num_commands) {
		/* deselect */
		exi_deselect(cmd);
		/* remove from queue */
		spin_lock_irqsave(&data->queue_lock,flags);
		list_del(&cmd->list);
		spin_unlock_irqrestore(&data->queue_lock,flags);
		/* reset pointer */
		data->cur_command = 0;
	}
	/* call the callback */
	if (subcmd->completion_routine) {
		subcmd->completion_routine(subcmd);
	}
}

static void exi_execute_queue(struct exi_locked_data *data)
{
	u32 channel;
	struct exi_command_group *cmd;
	struct exi_command *subcmd;
	
	/* execute the first command, but do not remove until done */
	cmd = (struct exi_command_group*)data->queue.next;
	channel   = (cmd->flags >> 16) & 3;
	
	/* is this the first item?  We must select it */
	if (data->cur_command == 0) {
		exi_select(cmd);
	}
	
	subcmd = cmd->commands + data->cur_command;
	/* execute transfer, determine if we can use DMA */
	if (((subcmd->len & (EXI_DMA_ALIGNMENT-1)) == 0) &&
	    (((u32)subcmd->data & (EXI_DMA_ALIGNMENT-1)) == 0)) {
		/* set state */
		atomic_set(&data->tc_interrupt,0);
		data->exi_state = EXI_WAITING_FOR_TC;
		/* do it */
		exi_dma_transfer(subcmd,channel);
	}
	else {
		exi_immediate_transfer(subcmd,channel);
		exi_complete_command(data,subcmd);
	}
}

u32 exi_synchronous_id(unsigned int channel,unsigned int device)
{
	struct exi_command_group cmd;
	struct exi_command sub;
	u16 write;
	u32 read;
	
	cmd.flags = (channel << 16) | 
		(EXI_DEVICE_0 << device) |
		EXI_FREQUENCY_3;
	
	write = 0;
	
	exi_select(&cmd);
	
	sub.flags = EXI_CMD_WRITE;
	sub.data  = &write;
	sub.len   = sizeof(write);
	exi_immediate_transfer(&sub,channel);

	sub.flags = EXI_CMD_READ;
	sub.data  = &read;
	sub.len   = sizeof(read);
	exi_immediate_transfer(&sub,channel);
	
	exi_deselect(&cmd);

	return read;
}

void exi_tasklet(unsigned long param)
{
	unsigned long flags;
	int empty;
	struct exi_locked_data *data = (struct exi_locked_data*)param;
	struct exi_command_group *cmd;
	struct exi_command *subcmd;
	
	while (1) {
		/* check if the queue has stuff in it */
		spin_lock_irqsave(&data->queue_lock,flags);
		empty = list_empty(&data->queue);
		spin_unlock_irqrestore(&data->queue_lock,flags);
		
		if (empty) {
			goto exit_tasklet;
		}
		/* stuff in queue, process it */
		switch (data->exi_state) {
		case EXI_IDLE:
			exi_execute_queue(data);
			break;
		case EXI_WAITING_FOR_TC:
			/* no interrupt yet? */
			if (!atomic_read(&data->tc_interrupt)) {
				goto exit_tasklet;
			}
			/* finish the operation */
			cmd = (struct exi_command_group*)data->queue.next;
			subcmd = cmd->commands + data->cur_command;
			/* invalidate the cache on a read */
			if (!(subcmd->flags & EXI_CMD_WRITE)) {
				invalidate_dcache_range((u32)subcmd->data,
							(u32)subcmd->data + 
							subcmd->len);
			}
			/* complete the operation */
			atomic_set(&data->tc_interrupt,0);
			exi_complete_command(data,subcmd);
			break;
		}
	}
 exit_tasklet:
	return;
}

irqreturn_t exi_bus_irq_handler(int irq,void *dev_id,struct pt_regs *regs)
{
	void * __iomem reg;
	u32 csr;
	u32 val;
	int channel;
	
	for (channel = 0; channel < EXI_MAX_CHANNELS; channel++)
	{
		/* find interrupt cause */
		reg = EXI_CSR(channel);
		csr = readl(reg);
		val = csr & (EXI_CSR_EXTINT | EXI_CSR_EXIINT | EXI_CSR_TCINT);
		if (!val)
			continue;
		/* ack */
		writel(csr,reg);
		
		if (csr & EXI_CSR_EXTINT)
		{
			/* insert happened */
			exi_bus_insert(channel,csr & EXI_CSR_EXT);
		}
		if ((csr & (EXI_CSR_TCINT | EXI_CSR_TCINTMASK)) == 
		    (EXI_CSR_TCINT | EXI_CSR_TCINTMASK))
		{
			/* there's a weird thing happening, we tend to get
			   these interrupts when the mask is off, so skip them
			   if the mask if off */
			
                        /* get the device data based on the channel */
			atomic_set(&exi_data[channel].tc_interrupt,1);
			tasklet_schedule(&exi_data[channel].tasklet);
			/* disable the TC interrupt */
			csr &= ~EXI_CSR_TCINTMASK;
			csr |= EXI_CSR_TCINT;
			writel(csr,reg);
		}
		if (csr & EXI_CSR_EXIINT)
		{
			/* fire the callback associated with the irq */
			if (irq_handlers[channel].func) {
				irq_handlers[channel].func(
					channel,irq_handlers[channel].param);
			}
		}
	}
	return IRQ_HANDLED;
}


void exi_add_command_group(struct exi_command_group *cmd,unsigned int count)
{
	unsigned long flags;
	unsigned int i;
	unsigned int freq;
	struct exi_driver *drv;
	struct exi_locked_data *data = (struct exi_locked_data*)cmd->dev->dev.platform_data;
	
	spin_lock_irqsave(&data->queue_lock,flags);
	/* add to the queue */
	for (i=0;i<count;++i) {
		/* modify the flags based on the device
		   pull out the frequency from the driver */
		drv = to_exi_driver(cmd->dev->dev.driver);
		freq = drv ? (drv->frequency << 24) : EXI_FREQUENCY_3;
		
		cmd->flags &= 0xFFFF;
		cmd->flags |= (cmd->dev->eid.channel << 16) |
			(EXI_DEVICE_0 << cmd->dev->eid.device) | freq;
		
		/* add to the list */
		list_add_tail(&cmd[i].list,&data->queue);
	}
	spin_unlock_irqrestore(&data->queue_lock,flags);
	/* now start the tasklet */
	tasklet_schedule(&data->tasklet);
}
