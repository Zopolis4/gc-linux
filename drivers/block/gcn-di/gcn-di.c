/*
 * drivers/block/gcn-di/gcn-di.c
 *
 * Nintendo GameCube DVD Interface driver
 * Copyright (C) 2005 The GameCube Linux Team
 * Copyright (C) 2005 Albert Herranz
 *
 * Portions based on previous work by Scream|CT.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/blkdev.h>
#include <linux/fcntl.h>
#include <linux/hdreg.h>
#include <linux/cdrom.h>

#include <asm/io.h>

#define DI_DEBUG 1

#define DRV_MODULE_NAME	"gcn-di"
#define DRV_DESCRIPTION	"Nintendo GameCube DVD Interface driver"
#define DRV_AUTHOR	"Albert Herranz"

static char di_driver_version[] = "0.1";

#define di_printk(level, format, arg...) \
	printk(level DRV_MODULE_NAME ": " format , ## arg)

#ifdef DI_DEBUG
#  define DBG(fmt, args...) \
          printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DBG(fmt, args...)
#endif


/*
 * Hardware.
 */
#define DI_IRQ			2

#define DI_DMA_ALIGN		0x1f /* 32 bytes */

#define DI_BASE			0xcc006000
#define DI_SIZE			0x40

#define DI_IO_BASE		((void __iomem *)DI_BASE)

/* DI Status Register */
#define DI_SR			0x00
#define  DI_SR_BRK		(1<<0)
#define  DI_SR_DEINTMASK	(1<<1)
#define  DI_SR_DEINT		(1<<2)
#define  DI_SR_TCINTMASK	(1<<3)
#define  DI_SR_TCINT		(1<<4)
#define  DI_SR_BRKINTMASK	(1<<5)
#define  DI_SR_BRKINT		(1<<6)

/* DI Cover Register */
#define DI_CVR			0x04
#define  DI_CVR_CVR		(1<<0)
#define  DI_CVR_CVRINTMASK	(1<<1)
#define  DI_CVR_CVRINT		(1<<2)

/* DI Command Buffers */
#define DI_CMDBUF0		0x08
#define DI_CMDBUF1		0x0c
#define DI_CMDBUF2		0x10

/* DI DMA Memory Address Register */
#define DI_MAR			0x14

/* DI DMA Transfer Length Register */
#define DI_LENGTH		0x18

/* DI Control Register */
#define DI_CR			0x1c
#define  DI_CR_TSTART		(1<<0)
#define  DI_CR_DMA		(1<<1)
#define  DI_CR_RW		(1<<2)

/* DI Immediate Data Buffer */
#define DI_DATA			0x20

/* DI Configuration Register */
#define DI_CFG			0x24


/* DI Sector Size */
#define DI_SECTOR_SHIFT		11
#define DI_SECTOR_SIZE		(1 << DI_SECTOR_SHIFT) /*2048*/
#define DI_MAX_SECTORS		712880


/* Driver Settings */
#define DI_NAME			"di"
#define DI_MAJOR		60

#define DI_COMMAND_TIMEOUT	20

#define KERNEL_SECTOR_SHIFT	9
#define KERNEL_SECTOR_SIZE	(1 << KERNEL_SECTOR_SHIFT) /*512*/


struct di_opcode {
	u16				op;
#define DI_OP(a,b)		(((u8)(a)<<8)|((u8)(b)))
#define   DI_DIR_READ		0x00
#define   DI_DIR_WRITE		DI_CR_RW
#define   DI_MODE_IMMED		0x00
#define   DI_MODE_DMA		DI_CR_DMA

	u32				cmdbuf0;
	u32				cmdbuf1;
	u32				cmdbuf2;
};

struct di_drive_info {
	u16				rev;
	u16				code;
	u32				date;
	u8				pad[0x18];
};

struct di_disk_id {
	u8				id[32];
};

struct di_drive_code {
	u32				address;
	size_t				len;
	void				*code;
};

struct di_device;

struct di_command {
	u16				opidx;

	u32				cmdbuf0;
	u32				cmdbuf1;
	u32				cmdbuf2;

	void				*data;
	size_t				len;

	dma_addr_t			dma_addr;
	size_t				dma_len;

	void				*done_data;
	void				(*done)(struct di_command *cmd);

	u32				result;

	struct di_device		*ddev;
};

#define di_command_ok(cmd)	((cmd)->result == DI_SR_TCINT)

struct di_device {
	spinlock_t			lock;

	int				irq;

	spinlock_t			io_lock;
	void __iomem			*io_base;

	struct di_command		*cmd;

	struct gendisk                  *disk;
	struct request_queue            *queue;
	spinlock_t			queue_lock;

	struct request                  *req;
	struct di_command		req_cmd;

	int				flags;
#define DI_INTEROPERABLE (1<<0)
#define DI_MEDIA_CHANGED (1<<1)
#define DI_START_QUEUE   (1<<2)

	unsigned long			nr_sectors;

#ifdef CONFIG_PROC_FS
	struct proc_dir_entry		*proc;
#endif /* CONFIG_PROC_FS */

	int				ref_count;

	struct platform_device		pdev;  /* must be last member */
};

/* get the di device given the platform device of a di device */
#define to_di_device(n) container_of(n,struct di_device,pdev)


/*
 * We do not accept original media with this driver, as there is currently no
 * general need for that.
 * If you ever develop an application (a media player for example) which works
 * with original media, just change di_accept_originals and recompile. 
 */
static const int di_accept_originals = 0;


/*
 * Drive firmware extensions.
 */

#define DI_DRIVE_CODE_BASE	0x40d000
#define DI_DRIVE_IRQ_VECTOR	0x00804c

static u32 generic_drive_code_entry_point = DI_DRIVE_CODE_BASE;
static struct di_drive_code generic_drive_code_trigger = {
	.address = DI_DRIVE_IRQ_VECTOR,
	.len = sizeof(generic_drive_code_entry_point),
	.code = (u8 *)&generic_drive_code_entry_point,
};

/*
 * Drive 04 (20020402) firmware extensions.
 */

#include "drive_20020402.h"

static struct di_drive_code drive_20020402[] = {
	[0] = {
		.address = DI_DRIVE_CODE_BASE,
		.len = sizeof(drive_20020402_firmware),
		.code = (u8 *)drive_20020402_firmware,
	},
};

/*
 * Drive 06 (XXX) firmware extensions.
 */

/* XXX TODO */

/*
 * Drive 08 (XXX) firmware extensions.
 */

/* XXX TODO */


/*
 * Drive operations table.
 * We just include here some of the available functions.
 */
#define CMDBUF(a,b,c,d) (((a)<<24)|((b)<<16)|((c)<<8)|(d))

static struct di_opcode di_opcodes[] = {

#define DI_OP_NOP		0
	[DI_OP_NOP] = {
		.op = DI_OP(DI_OP_NOP, 0),
		.cmdbuf0 = 0,
		.cmdbuf1 = 0,
		.cmdbuf2 = 0,
	},

#define DI_OP_INQ		(DI_OP_NOP+1)
	[DI_OP_INQ] = {
		.op = DI_OP(DI_OP_INQ, DI_DIR_READ | DI_MODE_DMA),
		.cmdbuf0 = 0x12000000,
		.cmdbuf1 = 0,
		.cmdbuf2 = 32,
	},

#define DI_OP_STOPMOTOR		(DI_OP_INQ+1)
	[DI_OP_STOPMOTOR] = {
		.op = DI_OP(DI_OP_STOPMOTOR, DI_DIR_READ | DI_MODE_IMMED),
		.cmdbuf0 = 0xe3000000,
		.cmdbuf1 = 0,
		.cmdbuf2 = 0,
	},

#define DI_OP_READDISKID	(DI_OP_STOPMOTOR+1)
	[DI_OP_READDISKID] = {
		.op = DI_OP(DI_OP_READDISKID, DI_DIR_READ | DI_MODE_DMA),
		.cmdbuf0 = 0xa8000040,
		.cmdbuf1 = 0,
		.cmdbuf2 = 32,
	},

#define DI_OP_READSECTOR	(DI_OP_READDISKID+1)
	[DI_OP_READSECTOR] = {
		.op = DI_OP(DI_OP_READSECTOR, DI_DIR_READ | DI_MODE_DMA),
		.cmdbuf0 = 0xa8000000,
		.cmdbuf1 = 0,
		.cmdbuf2 = 0,
	},

#define DI_OP_UNLOCK1		(DI_OP_READSECTOR+1)
	[DI_OP_UNLOCK1] = {
		.op = DI_OP(DI_OP_UNLOCK1, DI_DIR_READ | DI_MODE_IMMED),
		.cmdbuf0 = CMDBUF(0xff, 0x01, 'M', 'A'),
		.cmdbuf1 = CMDBUF('T', 'S', 'H', 'I'),
		.cmdbuf2 = CMDBUF('T', 'A', 0x02, 0x00),
	},

#define DI_OP_UNLOCK2		(DI_OP_UNLOCK1+1)
	[DI_OP_UNLOCK2] = {
		.op = DI_OP(DI_OP_UNLOCK2, DI_DIR_READ | DI_MODE_IMMED),
		.cmdbuf0 = CMDBUF(0xff, 0x00, 'D', 'V'),
		.cmdbuf1 = CMDBUF('D', '-', 'G', 'A'),
		.cmdbuf2 = CMDBUF('M', 'E', 0x03, 0x00),
	},

#define DI_OP_READMEM		(DI_OP_UNLOCK2+1)
	[DI_OP_READMEM] = {
		.op = DI_OP(DI_OP_READMEM, DI_DIR_READ | DI_MODE_IMMED),
		.cmdbuf0 = 0xfe010000,
		.cmdbuf1 = 0,
		.cmdbuf2 = 0x00010000,
	},

#define DI_OP_WRITEMEM		(DI_OP_READMEM+1)
	[DI_OP_WRITEMEM] = {
		.op = DI_OP(DI_OP_WRITEMEM, DI_DIR_READ | DI_MODE_DMA),
		.cmdbuf0 = 0xfe010100,
		.cmdbuf1 = 0,
		.cmdbuf2 = 0,
	},

#define DI_OP_MAXOP		DI_OP_WRITEMEM
};

#define DI_OP_CUSTOM		((u16)~0)


static void di_reset(struct di_device *ddev);

/*
 * Returns the operation code related data for a command.
 */
static inline struct di_opcode *di_get_opcode(struct di_command *cmd)
{
	BUG_ON(cmd->opidx > DI_OP_MAXOP && cmd->opidx != DI_OP_CUSTOM);

	if (cmd->opidx == DI_OP_CUSTOM) {
		return cmd->data;
	} else {
		return &di_opcodes[cmd->opidx];
	}
}

/*
 * Returns the operation code for a command.
 */
static inline u16 di_op(struct di_command *cmd)
{
	return di_get_opcode(cmd)->op;
}


/*
 * Basic initialization for all commands.
 */
static void di_op_basic(struct di_command *cmd,
			struct di_device *ddev, u16 opidx)
{
	struct di_opcode *opcode;

	memset(cmd, 0, sizeof(*cmd));
	cmd->ddev = ddev;
	cmd->opidx = opidx;
	opcode = di_get_opcode(cmd);
	if (opcode) {
		cmd->cmdbuf0 = opcode->cmdbuf0;
		cmd->cmdbuf1 = opcode->cmdbuf1;
		cmd->cmdbuf2 = opcode->cmdbuf2;
	}
}

/*
 * Builds an "Inquiry" command.
 */
static inline void di_op_inq(struct di_command *cmd,
			     struct di_device *ddev,
			     struct di_drive_info *drive_info)
{
	di_op_basic(cmd, ddev, DI_OP_INQ);
	cmd->data = drive_info;
	cmd->len = sizeof(*drive_info);
}

/*
 * Builds a "Stop Motor" command.
 */
static inline void di_op_stopmotor(struct di_command *cmd,
				   struct di_device *ddev)
{
	di_op_basic(cmd, ddev, DI_OP_STOPMOTOR);
}

/*
 * Builds a "Read Disc ID" command.
 */
static inline void di_op_readdiskid(struct di_command *cmd,
			      struct di_device *ddev,
			      struct di_disk_id *disk_id)
{
	di_op_basic(cmd, ddev, DI_OP_READDISKID);
	cmd->data = disk_id;
	cmd->len = sizeof(*disk_id);
}

/*
 * Builds a "Read Sector" command.
 */
static inline void di_op_readsector(struct di_command *cmd,
				    struct di_device *ddev,
				    u32 sector, void *data, size_t len)
{
	di_op_basic(cmd, ddev, DI_OP_READSECTOR);
	cmd->cmdbuf1 = sector;
	cmd->cmdbuf2 = len;
	cmd->data = data;
	cmd->len = len;
}

/*
 * Builds the first unlock command.
 */
static inline void di_op_unlock1(struct di_command *cmd,
				 struct di_device *ddev)
{
	di_op_basic(cmd, ddev, DI_OP_UNLOCK1);
}

/*
 * Builds the second unlock command.
 */
static inline void di_op_unlock2(struct di_command *cmd,
				 struct di_device *ddev)
{
	di_op_basic(cmd, ddev, DI_OP_UNLOCK2);
}

/*
 * Builds a "Read Memory" command.
 */
static inline void di_op_readmem(struct di_command *cmd,
				 struct di_device *ddev)
{
	di_op_basic(cmd, ddev, DI_OP_READMEM);
}

/*
 * Builds a "Write Memory" command.
 */
static inline void di_op_writemem(struct di_command *cmd,
				 struct di_device *ddev)
{
	di_op_basic(cmd, ddev, DI_OP_WRITEMEM);
}

/*
 * Builds a customized command.
 */
static inline void di_op_custom(struct di_command *cmd,
				struct di_device *ddev,
				struct di_opcode *opcode)
{
	di_op_basic(cmd, ddev, DI_OP_NOP);
	cmd->opidx = DI_OP_CUSTOM;
	cmd->data = opcode;
}


/*
 * Converts a request direction into a DMA data direction.
 */
static inline
enum dma_data_direction di_opidx_to_dma_dir(struct di_command *cmd)
{
	u16 op = di_op(cmd);

	if ((op & DI_DIR_WRITE)) {
		return DMA_TO_DEVICE;
	} else {
		return DMA_FROM_DEVICE;
	}
}

/*
 * Starts a DMA transfer.
 */
static void di_start_dma_transfer_raw(struct di_device *ddev,
				      dma_addr_t data, size_t len, int mode)
{
	void __iomem *io_base = ddev->io_base;
	u32 __iomem *sr_reg = io_base + DI_SR;
	unsigned long flags;

	BUG_ON((data & DI_DMA_ALIGN) != 0 ||
	       (len & DI_DMA_ALIGN) != 0);

	/* setup address and length of transfer */
	writel(len, io_base + DI_LENGTH);
	writel(data, io_base + DI_MAR);

	/* enable the Transfer Complete interrupt */
	spin_lock_irqsave(&ddev->io_lock, flags);
	writel(readl(sr_reg) | DI_SR_TCINTMASK, sr_reg);
	spin_unlock_irqrestore(&ddev->io_lock, flags);

	/* start the transfer */
	writel(DI_CR_TSTART | DI_CR_DMA | (mode&0x4), io_base + DI_CR);
}

/*
 * Internal. Busy-waits until a DMA transfer finishes or timeouts.
 */
static int __wait_for_dma_transfer_or_timeout(u32 __iomem *cr_reg,
					      int secs)
{
	unsigned long timeout = jiffies + secs*HZ;

	/* busy-wait for transfer complete */
	while(readl(cr_reg) & DI_CR_TSTART && time_before(jiffies, timeout)) {
		cpu_relax();
	}

	return (readl(cr_reg) & DI_CR_TSTART)?-EBUSY:0;
}

/*
 * Busy-waits until DMA transfers are finished.
 */
static void di_wait_for_dma_transfer_raw(struct di_device *ddev)
{
	u32 __iomem *cr_reg = ddev->io_base + DI_CR;
	u32 __iomem *sr_reg = ddev->io_base + DI_SR;
	unsigned long flags;

	/* we don't want TCINTs to disturb us while waiting */
	spin_lock_irqsave(&ddev->io_lock, flags);
	writel(readl(sr_reg) & ~DI_SR_TCINTMASK, sr_reg);
	spin_unlock_irqrestore(&ddev->io_lock, flags);

	/* if the drive got stuck, reset it */
	if (__wait_for_dma_transfer_or_timeout(cr_reg, DI_COMMAND_TIMEOUT)) {
		di_printk(KERN_ERR, "dvd stuck!\n");
		di_reset(ddev);
		ddev->flags |= DI_MEDIA_CHANGED;
	}

	/* ack the Transfer Complete interrupt */
	spin_lock_irqsave(&ddev->io_lock, flags);
	writel(readl(sr_reg) | DI_SR_TCINT, sr_reg);
	spin_unlock_irqrestore(&ddev->io_lock, flags);
}

/*
 * Outputs the command buffers, and optionally starts a transfer.
 */
static void di_prepare_command(struct di_command *cmd, int tstart)
{
	struct di_opcode *opcode = di_get_opcode(cmd);
	void __iomem *io_base = cmd->ddev->io_base;

	//DBG("buf0 = 0x%08x, buf1 = 0x%08x, buf2 = 0x%08x\n",
	//    cmd->cmdbuf0, cmd->cmdbuf1, cmd->cmdbuf2);

	writel(cmd->cmdbuf0, io_base + DI_CMDBUF0);
	writel(cmd->cmdbuf1, io_base + DI_CMDBUF1);
	writel(cmd->cmdbuf2, io_base + DI_CMDBUF2);

	if (tstart) {
		writel(DI_CR_TSTART | (opcode->op & 0x6), io_base + DI_CR);
	}
}

static void di_command_done(struct di_command *cmd);

/*
 * Starts a command by using the immediate mode.
 */
static int di_start_command(struct di_command *cmd)
{
	struct di_device *ddev = cmd->ddev;
	unsigned long flags;
	int retval = 1;

	spin_lock_irqsave(&ddev->lock, flags);

	BUG_ON(ddev->cmd);

	ddev->cmd = cmd;
	cmd->dma_len = 0;
	di_prepare_command(cmd, 1);

	spin_unlock_irqrestore(&ddev->lock, flags);

	return retval;
}

/*
 * Starts a command by using the DMA mode.
 */
static int di_start_dma_command(struct di_command *cmd)
{
	struct di_device *ddev = cmd->ddev;
	unsigned long flags;
	int retval = 1;

	spin_lock_irqsave(&ddev->lock, flags);

	BUG_ON(ddev->cmd);

	ddev->cmd = cmd;
	cmd->dma_len = cmd->len;
	cmd->dma_addr = dma_map_single(&ddev->pdev.dev,
				       cmd->data, cmd->len,
				       di_opidx_to_dma_dir(cmd));

	di_prepare_command(cmd, 0);
	di_start_dma_transfer_raw(ddev, cmd->dma_addr, cmd->dma_len,
				  di_op(cmd) & DI_DIR_WRITE);

	spin_unlock_irqrestore(&ddev->lock, flags);

	return retval;
}

/*
 * Called after a transfer is completed.
 */
static void di_complete_transfer(struct di_device *ddev, u32 result)
{
	struct di_command *cmd;
	unsigned long flags;

	spin_lock_irqsave(&ddev->lock, flags);

	//di_wait_for_dma_transfer_raw(ddev);

	cmd = ddev->cmd;
	if (cmd) {
		if (cmd->dma_len) {
			dma_unmap_single(&ddev->pdev.dev,
					 cmd->dma_addr, cmd->dma_len,
					 di_opidx_to_dma_dir(cmd));
		}
		ddev->cmd = NULL;
		spin_unlock_irqrestore(&ddev->lock, flags);
		cmd->result = result;
		di_command_done(cmd);
		if (ddev->flags & DI_START_QUEUE) {
			spin_lock(&ddev->queue_lock);
			ddev->flags &= ~DI_START_QUEUE;
			blk_start_queue(ddev->queue);
			spin_unlock(&ddev->queue_lock);
		}
		return;
	}

	spin_unlock_irqrestore(&ddev->lock, flags);
}

/*
 * Calls any done hooks.
 */
static void di_command_done(struct di_command *cmd)
{
	/* if specified, call the completion routine */
	if (cmd->done) {
		cmd->done(cmd);
	}
}

/*
 * Completion routine.
 */
static void di_wait_done(struct di_command *cmd)
{
	complete(cmd->done_data);
}

/*
 * Runs a command.
 */
static int di_run_command(struct di_command *cmd)
{
	struct di_opcode *opcode = di_get_opcode(cmd);
	int retval;

	if (!(opcode->op & DI_MODE_DMA)) {
		retval = di_start_command(cmd);
	} else {
		retval = di_start_dma_command(cmd);
	}
	return retval;
}

/*
 * Runs a command and waits.
 * Might sleep if called from user context.
 */
static int di_run_command_and_wait(struct di_command *cmd)
{
	DECLARE_COMPLETION(complete);

	cmd->done_data = &complete;
	cmd->done = di_wait_done;
	if (di_run_command(cmd) > 0) {
		wait_for_completion(&complete);
	}
	return cmd->result;
}

/*
 * Interrupt handler for DI interrupts.
 */
static irqreturn_t di_irq_handler(int irq, void *dev0, struct pt_regs *regs)
{
	struct di_device *ddev = dev0;
	void __iomem *io_base = ddev->io_base;
	u32 __iomem *sr_reg = io_base + DI_SR;
	u32 __iomem *cvr_reg = io_base + DI_CVR;
	u32 sr, cvr, reason, mask;
	unsigned long flags;

	spin_lock_irqsave(&ddev->io_lock, flags);

	sr = readl(sr_reg);
	mask = sr & (DI_SR_BRKINTMASK | DI_SR_TCINTMASK | DI_SR_DEINTMASK);
	reason = sr; // & (mask << 1);
	if (reason) {
		writel(sr | reason, sr_reg);
		spin_unlock_irqrestore(&ddev->io_lock, flags);

		if (reason & DI_SR_TCINT) {
			//DBG("TCINT\n");
			di_complete_transfer(ddev, DI_SR_TCINT);
		}
		if (reason & DI_SR_BRKINT) {
			DBG("BRKINT\n");
			di_complete_transfer(ddev, DI_SR_BRKINT);
		}
		if (reason & DI_SR_DEINT) {
			DBG("DEINT\n");
			di_complete_transfer(ddev, DI_SR_DEINT);
		}

		spin_lock_irqsave(&ddev->io_lock, flags);
	}

	cvr = readl(cvr_reg);
	mask = cvr & DI_CVR_CVRINTMASK;
	reason = cvr; // & (mask << 1);
	if ((reason & DI_CVR_CVRINT)) {
		writel(cvr | DI_CVR_CVRINT, cvr_reg);
		ddev->flags |= DI_MEDIA_CHANGED;
		DBG("dvd cover interrupt\n");
	}

	spin_unlock_irqrestore(&ddev->io_lock, flags);

	return IRQ_HANDLED;
}


/*
 * Patches drive addressable memory.
 */
static void di_patch_mem(struct di_device *ddev, u32 address,
			 void *data, size_t len)
{
	struct di_command cmd;
	struct di_opcode opcode;
	int chunk_size;
	const int max_chunk_size = 3 * sizeof(cmd.cmdbuf0);

	while(len > 0) {
		if (len > max_chunk_size)
			chunk_size = max_chunk_size;
		else
			chunk_size = len;

		di_op_writemem(&cmd, ddev);
		cmd.cmdbuf1 = address;
		cmd.cmdbuf2 = chunk_size << 16;
		di_run_command_and_wait(&cmd);

		opcode.op = DI_OP(DI_OP_CUSTOM, DI_DIR_READ | DI_MODE_IMMED);
		di_op_custom(&cmd, ddev, &opcode);
		memcpy(&cmd.cmdbuf0, data, chunk_size);
		di_run_command(&cmd);
		di_wait_for_dma_transfer_raw(ddev);
		di_complete_transfer(ddev, DI_SR_TCINT);

		address += chunk_size;
		data += chunk_size;
		len -= chunk_size;
	}
}

/*
 * Resets the drive (hard).
 */
static void di_reset(struct di_device *ddev)
{
	u32 __iomem *reset_reg = (u32 __iomem *)0xcc003024;
	u32 reset;
#define FLIPPER_RESET_DVD 0x00000004

	reset = readl(reset_reg);
	writel((reset & ~FLIPPER_RESET_DVD) | 1, reset_reg);
	mdelay(500);
	writel((reset | FLIPPER_RESET_DVD) | 1, reset_reg);
	mdelay(500);

	ddev->flags = DI_MEDIA_CHANGED;
}

/*
 * Runs a series of patches.
 */
static void di_patch(struct di_device *ddev,
		     struct di_drive_code *section, int nr_sections)
{
	while(nr_sections > 0) {
		di_patch_mem(ddev, section->address,
			     section->code, section->len);
		section++;
		nr_sections--;
	}
}

/*
 * Configures the drive to accept DVD-R media.
 */
static void di_make_interoperable(struct di_device *ddev)
{
	static struct di_drive_info drive_info
			 __attribute__ ((aligned (DI_DMA_ALIGN+1)));
	struct di_command cmd;

	/* enable the extended command set */
	di_op_unlock1(&cmd, ddev);
	di_run_command_and_wait(&cmd);
	di_op_unlock2(&cmd, ddev);
	di_run_command_and_wait(&cmd);

	/* get the drive model */
	memset(&drive_info, 0, sizeof(drive_info));
	di_op_inq(&cmd, ddev, &drive_info);
	di_run_command_and_wait(&cmd);

	di_printk(KERN_INFO, "drive_info: rev=%x, code=%x, date=%x\n",
		drive_info.rev, drive_info.code, drive_info.date);

	/* extend the firmware to allow use of normal media */
	di_printk(KERN_INFO, "loading drive %x extensions\n",
		  drive_info.date);

	switch(drive_info.date) {
		case 0x20020402:
			di_patch(ddev, drive_20020402,
				 ARRAY_SIZE(drive_20020402));
			di_patch(ddev, &generic_drive_code_trigger, 1);
			ddev->flags |= DI_INTEROPERABLE;
			break;
		default:
			di_printk(KERN_ERR, "sorry, drive %x is not yet"
				  " supported\n",
				  drive_info.date);
			break;
	}
}

/*
 * Prints the disk identifier.
 */
static void di_print_disk_id(struct di_disk_id *disk_id)
{
	di_printk(KERN_INFO, "disk_id = [%s]\n", disk_id->id);
}

/*
 * Determines media type and accepts accordingly.
 */
static int di_read_toc(struct di_device *ddev)
{
	static struct di_disk_id disk_id
			 __attribute__ ((aligned (DI_DMA_ALIGN+1)));
	struct di_command cmd;
	int nr_attempts = 2;
	int accepted_media = 0;
	int retval = 0;

	if ((ddev->flags & DI_MEDIA_CHANGED)) {
		di_reset(ddev);
	}

	memset(&disk_id, 0, sizeof(disk_id));
	while(nr_attempts > 0) {
		di_op_readdiskid(&cmd, ddev, &disk_id);
		di_run_command_and_wait(&cmd);
		if (di_command_ok(&cmd)) {
			if (disk_id.id[0]) {
				di_print_disk_id(&disk_id);
				if (!di_accept_originals) {
					di_printk(KERN_INFO, "sorry, original"
						  " media support is"
						  " disabled\n");
					break;
				}
			}

			if (!disk_id.id[0] || di_accept_originals) {
				accepted_media = 1;
				break;
			}
		}

		if (!(ddev->flags & DI_INTEROPERABLE))
			di_make_interoperable(ddev);

		--nr_attempts;
	}

	if (accepted_media) {
		ddev->nr_sectors = DI_MAX_SECTORS; /* in DVD sectors */
		ddev->flags &= ~DI_MEDIA_CHANGED;
	} else {
		ddev->nr_sectors = 0;
		retval = -ENOMEDIUM;
	}

	/* transform to kernel sectors */
	ddev->nr_sectors <<= (DI_SECTOR_SHIFT - KERNEL_SECTOR_SHIFT);

	set_capacity(ddev->disk, ddev->nr_sectors);
	return retval;
}


/*
 * Finishes a block layer request.
 */
static void di_request_done(struct di_command *cmd)
{
	struct di_device *ddev = cmd->ddev;
	struct request *req;
	unsigned long flags;

	spin_lock_irqsave(&ddev->io_lock, flags);

	req = ddev->req;
	ddev->req = NULL;

	spin_unlock_irqrestore(&ddev->io_lock, flags);

	if (req) {
		if (!end_that_request_first(req,
					    (cmd->result & DI_SR_TCINT)?1:0,
					    req->current_nr_sectors)) {
			add_disk_randomness(req->rq_disk);
			end_that_request_last(req);
		}
		spin_lock(&ddev->queue_lock);
		blk_start_queue(ddev->queue);
		spin_unlock(&ddev->queue_lock);
	}
}

/*
 * Processes a block layer request.
 */
static void di_do_request(request_queue_t *q)
{
	struct di_device *ddev = q->queuedata;
	struct di_command *cmd = &ddev->req_cmd;
	struct request *req;
	unsigned long start;
	unsigned long flags;
	size_t len;

	while ((req = elv_next_request(q))) {
		if (req->sector + req->current_nr_sectors > ddev->nr_sectors) {
			di_printk(KERN_ERR, "reading past end\n");
			end_request(req, 0);
			continue;
		}

		if (rq_data_dir(req) == WRITE) {
			di_printk(KERN_ERR, "write attempted\n");
			end_request(req, 0);
			continue;
		}

		if ((ddev->flags & DI_MEDIA_CHANGED)) {
			di_printk(KERN_ERR, "media changed, aborting\n");
			end_request(req, 0);
			continue;
		}

		spin_lock_irqsave(&ddev->io_lock, flags);

		/* we can schedule just a single request each time */
		if (ddev->req || ddev->cmd) {
			blk_stop_queue(q);
			if (ddev->cmd)
				ddev->flags |= DI_START_QUEUE;
			spin_unlock_irqrestore(&ddev->io_lock, flags);
			break;
		}

		blkdev_dequeue_request(req);

		/* ignore requests that we can't handle */
		if (!blk_fs_request(req)) {
			spin_unlock_irqrestore(&ddev->io_lock, flags);
			continue;
		}

		/* store the request being handled */
		ddev->req = req;
		blk_stop_queue(q);

		spin_unlock_irqrestore(&ddev->io_lock, flags);

		/* launch the corresponding read sector command */
		start = req->sector << KERNEL_SECTOR_SHIFT;
		len = req->current_nr_sectors << KERNEL_SECTOR_SHIFT;

		di_op_readsector(cmd, ddev, start >> 2,
				 req->buffer, len);
		cmd->done_data = cmd;
		cmd->done = di_request_done;
		di_run_command(cmd);
	}
}

/*
 * Opens the drive device.
 */
static int di_open(struct inode *inode, struct file *filp)
{
	struct di_device *ddev = inode->i_bdev->bd_disk->private_data;
	unsigned long flags;
	int retval = 0;

	if (filp->f_mode & FMODE_WRITE) {
		retval = -EROFS;
		goto out;
	}

	/* only allow a minor of 0 to be opened */
	if (iminor(inode)) {
		retval =  -ENODEV;
		goto out;
	}

        spin_lock_irqsave(&ddev->queue_lock, flags);

	/* honor exclusive open mode */
	if (ddev->ref_count == -1 ||
	    (ddev->ref_count && (filp->f_flags & O_EXCL))) {
		retval = -EBUSY;
		goto out_unlock;
	}

	check_disk_change(inode->i_bdev);
	if (!ddev->nr_sectors) {
		retval = -ENOMEDIUM;
		goto out_unlock;
	}

	if ((filp->f_flags & O_EXCL))
		ddev->ref_count = -1;
	else
		ddev->ref_count++;

out_unlock:
	spin_unlock_irqrestore(&ddev->queue_lock, flags);
out:
	return retval;

}

/*
 * Releases the drive device.
 */
static int di_release(struct inode *inode, struct file *filp)
{
	struct di_device *ddev = inode->i_bdev->bd_disk->private_data;
	struct di_command cmd;
	unsigned long flags;

	spin_lock_irqsave(&ddev->queue_lock, flags);

	if (ddev->ref_count > 0)
		ddev->ref_count--;
	else
		ddev->ref_count = 0;

	spin_unlock_irqrestore(&ddev->queue_lock, flags);

	if (ddev->ref_count == 0) {

		/* XXX check if command scheduler is busy */
		while(ddev->cmd)
			cpu_relax();

		di_op_stopmotor(&cmd, ddev);
		di_run_command_and_wait(&cmd);

		ddev->flags |= DI_MEDIA_CHANGED;
	}

	return 0;
}

/*
 * Checks if media is still valid.
 */
static int di_revalidate_disk(struct gendisk *disk)
{
	struct di_device *ddev = disk->private_data;
	di_read_toc(ddev);
	return 0;
}

/*
 * Checks if media changed.
 */
static int di_media_changed(struct gendisk *disk)
{
	struct di_device *ddev = disk->private_data;
	return (ddev->flags & DI_MEDIA_CHANGED) ? 1 : 0;
}

/*
 * Ioctl. Specific CDROM stuff is pending support.
 */
static int di_ioctl(struct inode *inode, struct file *filp,
		    unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case CDROMMULTISESSION:
		/* struct cdrom_multisession */
		break;
	case CDROMSTART:
		break;
	case CDROMSTOP:
		break;
	case CDROMREADTOCHDR:
		/* struct cdrom_tochdr */
		break;
	case CDROMREADTOCENTRY:
		/* struct cdrom_tocentry */
		break;
	case CDROMREADMODE2:
	case CDROMREADMODE1:
	case CDROMREADRAW:
		/* struct cdrom_read (1-2048, 2-2336,RAW-2352) */
		break;
	case CDROM_GET_MCN:
		/* retrieve the universal product code */
		/* struct cdrom_mcn */
		break;
	case CDROMRESET:
		/* reset the drive */
		break;
	case BLKRAGET:
	case BLKFRAGET:
	case BLKROGET:
	case BLKBSZGET:
	case BLKSSZGET:
	case BLKSECTGET:
	case BLKGETSIZE:
	case BLKGETSIZE64:
	case BLKFLSBUF:
		return ioctl_by_bdev(inode->i_bdev,cmd,arg);
	default:
		return -ENOTTY;
	}
	return -ENOTTY;
}

static struct block_device_operations di_fops = {
	.owner = THIS_MODULE,
	.open = di_open,
	.release = di_release,
	.revalidate_disk = di_revalidate_disk,
	.media_changed = di_media_changed,
	.ioctl = di_ioctl,
};

/*
 * Quiesces the hardware to a calm and known state.
 */
static void di_quiesce(struct di_device *ddev)
{
	void __iomem *io_base = ddev->io_base;
	u32 __iomem *cr_reg = io_base + DI_CR;
	u32 __iomem *sr_reg = io_base + DI_SR;
	u32 __iomem *cvr_reg = io_base + DI_CVR;
	u32 sr, cvr;
	unsigned long flags;

	spin_lock_irqsave(&ddev->io_lock, flags);

	/* ack and mask dvd io interrupts */
	sr = readl(sr_reg);
	sr |= DI_SR_BRKINT | DI_SR_TCINT | DI_SR_DEINT;
	sr &= ~(DI_SR_BRKINTMASK | DI_SR_TCINTMASK | DI_SR_DEINTMASK);
	writel(sr, sr_reg);

	/* ack and mask dvd cover interrupts */
	cvr = readl(cvr_reg);
	writel((cvr | DI_CVR_CVRINT) & ~DI_CVR_CVRINTMASK, cvr_reg);

	spin_unlock_irqrestore(&ddev->io_lock, flags);

	/* busy-wait for transfer complete */
	__wait_for_dma_transfer_or_timeout(cr_reg, DI_COMMAND_TIMEOUT);
}

/*
 * Initializes the hardware.
 */
static int di_init_irq(struct di_device *ddev)
{
	void __iomem *io_base = ddev->io_base;
	u32 __iomem *sr_reg = io_base + DI_SR;
	u32 __iomem *cvr_reg = io_base + DI_CVR;
	u32 sr, cvr;
	struct di_command cmd;
	unsigned long flags;
	int retval;

	ddev->flags |= DI_MEDIA_CHANGED;

	/* request interrupt */
	retval = request_irq(ddev->irq, di_irq_handler, 0,
			     DRV_MODULE_NAME, ddev);
	if (retval) {
		di_printk(KERN_ERR, "request of irq%d failed\n", ddev->irq);
		goto out;
	}

	spin_lock_irqsave(&ddev->io_lock, flags);

	sr = readl(sr_reg);
	sr |= DI_SR_BRKINT | DI_SR_TCINT | DI_SR_DEINT;
	sr |= DI_SR_BRKINTMASK | DI_SR_TCINTMASK | DI_SR_DEINTMASK;
	writel(sr, sr_reg);

	cvr = readl(cvr_reg);
	writel(cvr | DI_CVR_CVRINT | DI_CVR_CVRINTMASK, cvr_reg);

	spin_unlock_irqrestore(&ddev->io_lock, flags);

	/* stop DVD motor */
	di_op_stopmotor(&cmd, ddev);
	di_run_command_and_wait(&cmd);

out:
	return retval;
}

/*
 * Relinquishes control of the haardware.
 */
static void di_exit_irq(struct di_device *ddev)
{
	struct di_command cmd;

	/* stop DVD motor */
	di_op_stopmotor(&cmd, ddev);
	di_run_command_and_wait(&cmd);

	di_quiesce(ddev);

	free_irq(ddev->irq, ddev);
}


/*
 * Initializes the block layer interfaces.
 */
static int di_init_blk_dev(struct di_device *ddev)
{
	struct gendisk *disk;
	struct request_queue *queue;
	int retval;

	spin_lock_init(&ddev->lock);
	spin_lock_init(&ddev->io_lock);

	ddev->ref_count = 0;

	retval = register_blkdev(DI_MAJOR, DI_NAME);
	if (retval) {
		di_printk(KERN_ERR, "error registering major %d\n", DI_MAJOR);
		goto err_register_blkdev;
	}

	retval = -ENOMEM;
	spin_lock_init(&ddev->queue_lock);
	queue = blk_init_queue(di_do_request, &ddev->queue_lock);
	if (!queue) {
		di_printk(KERN_ERR, "error initializing queue\n");
		goto err_blk_init_queue;
	}

	blk_queue_hardsect_size(queue, DI_SECTOR_SIZE);
	blk_queue_dma_alignment(queue, DI_DMA_ALIGN);
	blk_queue_max_phys_segments(queue, 1);
	blk_queue_max_hw_segments(queue, 1);
	queue->queuedata = ddev;
	ddev->queue = queue;

	disk = alloc_disk(1);
	if (!disk) {
		di_printk(KERN_ERR, "error allocating disk\n");
		goto err_alloc_disk;
	}

	disk->major = DI_MAJOR;
	disk->first_minor = 0;
	disk->fops = &di_fops;
	strcpy(disk->disk_name, DI_NAME);
	strcpy(disk->devfs_name, disk->disk_name);
	disk->queue = ddev->queue;
	disk->private_data = ddev;
	ddev->disk = disk;

	set_disk_ro(ddev->disk, 1);
	add_disk(ddev->disk);

	retval = 0;
	goto out;

err_alloc_disk:
        blk_cleanup_queue(ddev->queue);
err_blk_init_queue:
        unregister_blkdev(DI_MAJOR, DI_NAME);
err_register_blkdev:
out:
	return retval;
}

/*
 * Exits the block layer interfaces.
 */
static void di_exit_blk_dev(struct di_device *ddev)
{
	if (ddev->disk) {
		del_gendisk(ddev->disk);
		put_disk(ddev->disk);
	}
	if (ddev->queue)
		blk_cleanup_queue(ddev->queue);
	unregister_blkdev(DI_MAJOR, DI_NAME);
}

/*
 * Initializes /proc filesystem support.
 */
static int di_init_proc(struct di_device *ddev)
{
#ifdef CONFIG_PROC_FS
#endif /* CONFIG_PROC_FS */
	return 0;
}

/*
 * Exits /proc filesystem support.
 */
static void di_exit_proc(struct di_device *ddev)
{
#ifdef CONFIG_PROC_FS
#endif /* CONFIG_PROC_FS */
}


/*
 * Initializes the device.
 */
static int di_init(struct di_device *ddev, struct resource *mem, int irq)
{
	int retval;

	cpu_to_le32s(&generic_drive_code_entry_point);

	memset(ddev, 0, sizeof(*ddev) - sizeof(ddev->pdev));

	ddev->io_base = (void __iomem *)mem->start;
	ddev->irq = irq;

	retval = di_init_blk_dev(ddev);
	if (!retval) {
		retval = di_init_irq(ddev);
		if (retval) {
			di_exit_blk_dev(ddev);
		} else {
			di_init_proc(ddev);
		}
	}
	return retval;
}

/*
 * Exits the device.
 */
static void di_exit(struct di_device *ddev)
{
        di_exit_blk_dev(ddev);
        di_exit_irq(ddev);
	di_exit_proc(ddev);
}


/*
 * Needed for platform devices.
 */
static void di_dev_release(struct device *dev)
{
}

/*
 * Set of resources used by the disk interface device.
 */
static struct resource di_resources[] = {
        [0] = {
                .start = DI_BASE,
                .end = DI_BASE + DI_SIZE -1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = DI_IRQ,
                .end = DI_IRQ,
                .flags = IORESOURCE_IRQ,
        },
};


/*
 * The disk interface device.
 */
static struct di_device di_device = {
	.pdev = {
		.name = DI_NAME,
		.id = 0,
		.num_resources = ARRAY_SIZE(di_resources),
		.resource = di_resources,
		.dev = {
			.release = di_dev_release,
		},
	},
};


/*
 * Drive model probe function for our device.
 */
static int di_probe(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct di_device *ddev = to_di_device(pdev);
	struct resource *mem;
	int irq;
	int retval;

	retval = -ENODEV;
	irq = platform_get_irq(pdev, 0);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem) {
		retval = di_init(ddev, mem, irq);
	}
	return retval;
}

/*
 * Drive model remove function for our device.
 */
static int di_remove(struct device *device)
{
        struct platform_device *pdev = to_platform_device(device);
        struct di_device *ddev = to_di_device(pdev);
                                                                                
        di_exit(ddev);
                                                                                
        return 0;
}

/*
 * Drive model shutdown function for our device.
 */
static void di_shutdown(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct di_device *ddev = to_di_device(pdev);

	di_quiesce(ddev);
}


/*
 * The disk interface driver.
 */
static struct device_driver di_driver = {
	.name = DI_NAME,
	.bus = &platform_bus_type,
	.probe = di_probe,
	.remove = di_remove,
	.shutdown = di_shutdown,
};

/*
 * Module initialization routine.
 */
static int __init di_init_module(void)
{
	int retval = 0;

	di_printk(KERN_INFO, "%s - version %s\n", DRV_DESCRIPTION,
		  di_driver_version);

	retval = driver_register(&di_driver);
	if (!retval) {
		retval = platform_device_register(&di_device.pdev);
	}

	return retval;
}

/*
 * Module de-initialization routine.
 */
static void __exit di_exit_module(void)
{
        platform_device_unregister(&di_device.pdev);
        driver_unregister(&di_driver);
}

module_init(di_init_module);
module_exit(di_exit_module);

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_LICENSE("GPL");

