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

#include <asm/setup.h>
#include <asm/bitops.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>
#include <linux/interrupt.h>

#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/hdreg.h>	/* HDIO_GETGEO */

#define ARAM_MAJOR Z2RAM_MAJOR

#define TRUE                  (1)
#define FALSE                 (0)

//#define ARAM_DBG      printk
#define ARAM_DBG(format, arg...); { }

//#define RAMDISK
#undef RAMDISK

#ifdef RAMDISK
unsigned char *RAMDISKBuffer;
#define ARAM_BUFFERSIZE 10*1024
#else
#define ARAM_BUFFERSIZE 14*1024*1024
#define ARAM_SOUNDMEMORYOFFSET	1024*1024
	//#define ARAM_BLOCKSIZE 1024
#endif

static int current_device = -1;
static spinlock_t aram_lock = SPIN_LOCK_UNLOCKED;

extern void flush_cache(void *start, unsigned int len);

static struct block_device_operations aram_fops;
static struct gendisk *aram_gendisk;

#define AR_DMA_MMADDR_H			*(unsigned short*)0xCC005020
#define AR_DMA_MMADDR_L			*(unsigned short*)0xCC005022
#define AR_DMA_ARADDR_H			*(unsigned short*)0xCC005024
#define AR_DMA_ARADDR_L			*(unsigned short*)0xCC005026
#define AR_DMA_CNT_H			*(unsigned short*)0xCC005028
#define AR_DMA_CNT_L			*(unsigned short*)0xCC00502A
#define AI_DSP_STATUS	 		*(volatile unsigned short*)0xCC00500A

#define ARAM_READ				1
#define ARAM_WRITE				0

void ARAM_StartDMA(unsigned long mmAddr, unsigned long arAddr,
		   unsigned long length, unsigned long type)
{

	//printk("ARAM DMA copy -> %08x - %08x  - %d  %d\n",mmAddr,arAddr,length,type);
	AR_DMA_MMADDR_H = mmAddr >> 16;
	AR_DMA_MMADDR_L = mmAddr & 0xFFFF;
	AR_DMA_ARADDR_H = arAddr >> 16;
	AR_DMA_ARADDR_L = arAddr & 0xFFFF;
	AR_DMA_CNT_H = (type << 15) | (length >> 16);
	AR_DMA_CNT_L = length & 0xFFFF;

	// We wait, until DMA finished
	while (AI_DSP_STATUS & 0x200) ;
}

/*
 	echo YUHUUhello1234567890hello12345678901234567890CCC > /dev/aram 
 	dd if=/dev/aram 
 	cat /dev/aram  | wc -c
*/
static void do_aram_request(request_queue_t * q)
{
	struct request *req;
	blk_stop_queue(q);
	spin_lock(&aram_lock);

	while ((req = elv_next_request(q)) != NULL) {
		unsigned long start = req->sector << 9;
		unsigned long len = req->current_nr_sectors << 9;

		if (start + len > ARAM_BUFFERSIZE) {
			printk(KERN_ERR DEVICE_NAME
			       ": bad access: block=%lu, count=%u\n",
			       (unsigned long)req->sector,
			       req->current_nr_sectors);
			end_request(req, 0);
			continue;
		}
#ifdef RAMDISK
		if (rq_data_dir(req) == READ) {
			memcpy(req->buffer, (char *)&RAMDISKBuffer[start], len);
		} else {
			memcpy((char *)&RAMDISKBuffer[start], req->buffer, len);
		}
#else
		if (rq_data_dir(req) == READ) {
			//memset(req->buffer,0,len);
			//flush_dcache_range((unsigned long)req->buffer,(unsigned long)req->buffer + len);
			ARAM_StartDMA((unsigned long)req->buffer,
				      start + ARAM_SOUNDMEMORYOFFSET, len,
				      ARAM_READ);
			//flush_dcache_range((unsigned long)req->buffer,(unsigned long)req->buffer + len);
			invalidate_dcache_range((unsigned long)req->buffer,
						(unsigned long)req->buffer +
						len);
		} else {
			flush_dcache_range((unsigned long)req->buffer,
					   (unsigned long)req->buffer + len);
			ARAM_StartDMA((unsigned long)req->buffer,
				      start + ARAM_SOUNDMEMORYOFFSET, len,
				      ARAM_WRITE);
		}
#endif

		end_request(req, 1);
	}

	spin_unlock(&aram_lock);
	blk_start_queue(q);

}

static int aram_open(struct inode *inode, struct file *filp)
{
	ARAM_DBG("A-RAM Open device\n");

	int device;
	int rc = -ENOMEM;

	device = iminor(inode);

	if (current_device != -1 && current_device != device) {
		rc = -EBUSY;
		goto err_out;
	}
#ifdef RAMDISK
	if (current_device == -1) {
		current_device = device;
		set_capacity(aram_gendisk, ARAM_BUFFERSIZE >> 9);
	}
#endif
	return 0;

      err_out:
	ARAM_DBG("A-RAM Open device Error %d\n", rc);

	return rc;

}

/*
	mkfs.minix /dev/aram 
	mount -t minix /dev/aram /mnt/

	dd if=/dev/urandom bs=1M count=10 > /mnt/test.bin   
	dd if=/dev/urandom bs=1M count=1 > /mnt/test1.bin

	
*/
static int aram_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	ARAM_DBG("A-RAM IOCTL\n");

	if (cmd == HDIO_GETGEO) {
		struct hd_geometry geo;
		/*
		 * get geometry: we have to fake one...  trim the size to a
		 * multiple of 2048 (1M): tell we have 32 sectors, 64 heads,
		 * whatever cylinders.
		 */
		geo.heads = 64;
		geo.sectors = 32;
		geo.start = 0;
		geo.cylinders = ARAM_BUFFERSIZE / (geo.heads * geo.sectors);

		if (copy_to_user((void *)arg, &geo, sizeof(geo)))
			return -EFAULT;
		return 0;
	}

	return -EINVAL;
}

static int aram_release(struct inode *inode, struct file *filp)
{
	ARAM_DBG("A-RAM Close device\n");
	if (current_device == -1)
		return 0;

	return 0;
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

static struct request_queue *aram_queue;

#if 0
static irqreturn_t aram_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	printk("interrupt received\n");
	return 0;
}
#endif

int __init aram_init(void)
{
	int ret;

	printk("A-Ram Block Device Driver Init\n");

	ret = -EBUSY;
	if (register_blkdev(ARAM_MAJOR, DEVICE_NAME))
		goto err;

	ret = -ENOMEM;
	aram_gendisk = alloc_disk(1);
	if (!aram_gendisk)
		goto out_disk;

	aram_queue = blk_init_queue(do_aram_request, &aram_lock);
	if (!aram_queue)
		goto out_queue;

	aram_gendisk->major = ARAM_MAJOR;
	aram_gendisk->first_minor = 0;
	aram_gendisk->fops = &aram_fops;
	sprintf(aram_gendisk->disk_name, "aram");
	strcpy(aram_gendisk->devfs_name, aram_gendisk->disk_name);

	aram_gendisk->queue = aram_queue;
	set_capacity(aram_gendisk, ARAM_BUFFERSIZE >> 9);

	add_disk(aram_gendisk);
	spin_lock_init(&aram_lock);

#if 0
	ret =
	    request_irq(5, aram_interrupt, 0, aram_gendisk->disk_name,
			aram_gendisk);
	if (ret) {
		//BBA_DBG(KERN_ERR "%s: unable to get IRQ %d\n", dev->name, dev->irq);
		return ret;
	}
#endif

#define AUDIO_DSP_CONTROL   *(volatile u_int16_t *)(0xCC00500a)
#define  AI_CSR_ARINTMASK   (1<<6)
	AUDIO_DSP_CONTROL &= ~AI_CSR_ARINTMASK;

#ifdef RAMDISK
	RAMDISKBuffer = kmalloc(ARAM_BUFFERSIZE, GFP_KERNEL);
#endif

	return 0;

      out_queue:
	put_disk(aram_gendisk);
      out_disk:
	unregister_blkdev(ARAM_MAJOR, DEVICE_NAME);
      err:
	return ret;
}

void __exit aram_cleanup(void)
{

	blk_unregister_region(MKDEV(ARAM_MAJOR, 0), 256);
	if (unregister_blkdev(ARAM_MAJOR, DEVICE_NAME) != 0)
		printk(KERN_ERR DEVICE_NAME ": unregister of device failed\n");

	del_gendisk(aram_gendisk);
	put_disk(aram_gendisk);
	blk_cleanup_queue(aram_queue);

#ifdef RAMDISK
	kfree(RAMDISKBuffer);
#endif

	return;
}

MODULE_LICENSE("GPL");
module_init(aram_init);
module_exit(aram_cleanup);
