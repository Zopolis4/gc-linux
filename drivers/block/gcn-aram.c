/*
 * drivers/block/gcn-aram.c
 *
 * Nintendo GameCube ARAM block device
 * Copyright (C) 2004 The GameCube Linux Team
 *
 * Based on previous work by Franz Lehner, apparently inspired on:
 * z2ram.c
 * Copyright (C) 1994 by Ingo Wilken (Ingo.Wilken@informatik.uni-oldenburg.de)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#define DEVICE_NAME "ARAM"

#include <linux/major.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <asm/setup.h>
#include <asm/bitops.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>
#include <linux/interrupt.h>

#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/hdreg.h>	/* HDIO_GETGEO */

#include <asm/io.h>

#define ARAM_MAJOR Z2RAM_MAJOR

#define HARD_SECTOR_SIZE 512

//#define ARAM_DBG      printk
#define ARAM_DBG(format, arg...); { }

#define ARAM_SOUNDMEMORYOFFSET 0
#define ARAM_BUFFERSIZE (16*1024*1024 - ARAM_SOUNDMEMORYOFFSET)

#define AI_DSP_CSR          (void* __iomem)0xCC00500A
#define  AI_CSR_PIINT       (1<<1)
#define  AI_CSR_ARINT       (1<<5)
#define  AI_CSR_ARINTMASK   (1<<6)
#define  AI_CSR_DSPINT      (1<<7)
#define  AI_CSR_DSPINTMASK  (1<<8)
#define  AI_CSR_DMA_STATUS  (1<<9)

#define AR_DMA_MMADDR        (void* __iomem)0xCC005020
#define AR_DMA_ARADDR	     (void* __iomem)0xCC005024
#define AR_DMA_CNT	     (void* __iomem)0xCC005028

#define ARAM_READ				(1 << 31)
#define ARAM_WRITE				0

#define ARAM_DMA_ALIGNMENT_MASK                 0x1F

#define ARAM_IRQ  6
#define IRQ_PARAM (void*)0xFFFFFFFA

static spinlock_t aram_lock = SPIN_LOCK_UNLOCKED;
static struct block_device_operations aram_fops;
static struct gendisk *aram_gendisk;
static struct request_queue *aram_queue;
static struct request * volatile irq_request;
static u32 refCount;

static inline void ARAM_StartDMA(unsigned long mmAddr, unsigned long arAddr,
				 unsigned long length, unsigned long type)
{
	writel(mmAddr,AR_DMA_MMADDR);
	writel(arAddr,AR_DMA_ARADDR);
	writel(type | length,AR_DMA_CNT);
}

static irqreturn_t aram_irq(int irq,void *dev_id,struct pt_regs *regs)
{
	unsigned long flags;
	unsigned long len;
	struct request *req;

	if (readw(AI_DSP_CSR) & AI_CSR_ARINT) {
		/* ack the int */
		local_irq_save(flags);
		writew(readw(AI_DSP_CSR) | AI_CSR_ARINT,AI_DSP_CSR);
		local_irq_restore(flags);
		
		/* now process */
		spin_lock_irqsave(&aram_lock,flags);
		if ((req = irq_request)) {
			len = req->current_nr_sectors << 9;
			/* invalidate cache on read */
			if (rq_data_dir(req) == READ) {
				invalidate_dcache_range(
					(u32)req->buffer,
					(u32)req->buffer + len);
			}
			/* complete request */
			if (!end_that_request_first(req,1,
						    req->current_nr_sectors)) {
				add_disk_randomness(req->rq_disk);
				end_that_request_last(req);
			}
			else {
				printk(KERN_ERR DEVICE_NAME " device still thinks there are requests but DMA has finished\n");
			}
			irq_request = NULL;
			/* start queue back up */
			blk_start_queue(aram_queue);
		}
		else {
			printk(KERN_ERR DEVICE_NAME " received an interrupt but no irq_request set\n");
		}
		spin_unlock_irqrestore(&aram_lock,flags);
		/* return handled */
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}


static void do_aram_request(request_queue_t * q)
{
	struct request *req;
	unsigned long start;
	unsigned long len;
	
	/*unsigned long flags; */
	
	while ((req = elv_next_request(q))) {
		/* get length */
		start = req->sector << 9;
		len = req->current_nr_sectors << 9;
		
		if ((start + len) > ARAM_BUFFERSIZE) {
			printk(KERN_ERR DEVICE_NAME
			       ": bad access: block=%lu, count=%lu\n",
			       (unsigned long)req->sector,len);
			end_request(req, 0);
			continue;
		}
		else if (irq_request) {	/* already scheduled? */
			blk_stop_queue(q);
			return;
		}
		/* dequeue */
		blkdev_dequeue_request(req);
		blk_stop_queue(q);
		irq_request = req;
		/* schedule DMA */
		if (rq_data_dir(req) == READ) {
			ARAM_StartDMA((unsigned long)req->buffer,
				      start + ARAM_SOUNDMEMORYOFFSET, len,
				      ARAM_READ);
		} 
		else {
			flush_dcache_range((unsigned long)req->buffer,
					   (unsigned long)req->buffer + len);
			ARAM_StartDMA((unsigned long)req->buffer,
				      start + ARAM_SOUNDMEMORYOFFSET, len,
				      ARAM_WRITE);
		}
		return;
	}
}

static int aram_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	
	ARAM_DBG("A-RAM Open device\n");
	/* only allow a minor of 0 to be opened */
	if (iminor(inode))
		return -ENODEV;
	/* allow multiple people to open this file */
	spin_lock_irqsave(&aram_lock,flags);
	++refCount;
	spin_unlock_irqrestore(&aram_lock,flags);
	return 0;
}

static int aram_release(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	ARAM_DBG("A-RAM Close device\n");
	/* lower ref count */
	spin_lock_irqsave(&aram_lock,flags);
	--refCount;
	spin_unlock_irqrestore(&aram_lock,flags);
	return 0;
}

static int aram_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	struct hd_geometry geo;
	
	ARAM_DBG("A-RAM IOCTL\n");
	
	switch (cmd) {
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
	case HDIO_GETGEO:
		/* fake the entries */
		geo.heads = 32;
		geo.sectors = 32;
		geo.start = 0;
		geo.cylinders = ARAM_BUFFERSIZE / (geo.heads * geo.sectors);
		if (copy_to_user((void __user*)arg,&geo,sizeof(geo)))
			return -EFAULT;
		return 0;
	default:
		return -ENOTTY;
	}
}

static int aram_revalidate(struct gendisk *disk)
{
	set_capacity(disk, ARAM_BUFFERSIZE >> 9);
	return 0;
}

static struct block_device_operations aram_fops = {
	.owner = THIS_MODULE,
	.open = aram_open,
	.release = aram_release,
	.revalidate_disk = aram_revalidate,
	.ioctl = aram_ioctl,
};



int __init aram_init(void)
{
	int ret;
	unsigned long flags;

	/* get the irq */
	if ((ret=request_irq(ARAM_IRQ,aram_irq,SA_SHIRQ,"ARAM",IRQ_PARAM)))
		goto out_irq;
	
	if ((ret=register_blkdev(ARAM_MAJOR, DEVICE_NAME)))
		goto out_blkdev;
	
	ret = -ENOMEM;
	aram_gendisk = alloc_disk(1);
	if (!aram_gendisk)
		goto out_disk;
	
	spin_lock_init(&aram_lock);
	
	aram_queue = blk_init_queue(do_aram_request, &aram_lock);
	if (!aram_queue)
		goto out_queue;

	aram_gendisk->major = ARAM_MAJOR;
	aram_gendisk->first_minor = 0;
	aram_gendisk->fops = &aram_fops;
	strcpy(aram_gendisk->disk_name, "aram");
	strcpy(aram_gendisk->devfs_name, aram_gendisk->disk_name);

	aram_gendisk->queue = aram_queue;

	/* we can only have one segment at a time */
	blk_queue_max_phys_segments(aram_queue,1);
	blk_queue_max_hw_segments(aram_queue,1);
	/* make sectors equal to the pagesize */
	blk_queue_hardsect_size(aram_queue,HARD_SECTOR_SIZE);
	/* set the DMA alignment */
	blk_queue_dma_alignment(aram_queue,ARAM_DMA_ALIGNMENT_MASK);
	
	set_capacity(aram_gendisk, ARAM_BUFFERSIZE >> 9);
	add_disk(aram_gendisk);

	/* lock this since audio driver might be using it */
	local_irq_save(flags);
	writew(readw(AI_DSP_CSR) | AI_CSR_ARINTMASK,AI_DSP_CSR);
	local_irq_restore(flags);

	refCount = 0;

	return 0;

 out_queue:
	del_gendisk(aram_gendisk);
	put_disk(aram_gendisk);
 out_disk:
	unregister_blkdev(ARAM_MAJOR, DEVICE_NAME);
 out_blkdev:
	free_irq(ARAM_IRQ,IRQ_PARAM);
 out_irq:
	return ret;
}

void __exit aram_cleanup(void)
{
	free_irq(ARAM_IRQ,IRQ_PARAM);

	blk_unregister_region(MKDEV(ARAM_MAJOR, 0), 256);
	
	if (unregister_blkdev(ARAM_MAJOR, DEVICE_NAME) != 0)
		printk(KERN_ERR DEVICE_NAME ": unregister of device failed\n");
	
	del_gendisk(aram_gendisk);
	put_disk(aram_gendisk);
	blk_cleanup_queue(aram_queue);
	
	return;
}

MODULE_AUTHOR("Todd Jeffreys <todd@voidpointer.org>");
MODULE_DESCRIPTION("Gamecube ARAM block driver");
MODULE_LICENSE("GPL");
module_init(aram_init);
module_exit(aram_cleanup);
