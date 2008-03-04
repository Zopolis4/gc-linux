/*
 * drivers/block/rvl-mem2.c
 *
 * Nintendo Wii MEM2 block driver
 * Copyright (C) 2008 The GameCube Linux Team
 * Copyright (C) 2008 Albert Herranz
 *
 * Based on gcn-aram.c.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/major.h>
#include <linux/platform_device.h>
#include <linux/blkdev.h>
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/hdreg.h>	/* HDIO_GETGEO */
#include <asm/io.h>


#define DRV_MODULE_NAME "rvl-mem2"
#define DRV_DESCRIPTION "Nintendo Wii MEM2 block driver"
#define DRV_AUTHOR      "Albert Herranz"

static char mem2_driver_version[] = "0.1";

#define mem2_printk(level, format, arg...) \
	printk(level DRV_MODULE_NAME ": " format , ## arg)

#ifdef MEM2_DEBUG
#  define DBG(fmt, args...) \
          printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DBG(fmt, args...)
#endif

/*
 * Hardware.
 */

#define MEM2_START	0x10000000
#define MEM2_SIZE	(52*1024*1024) /* 52MB */
#define MEM2_END	(MEM2_START + MEM2_SIZE - 1)

/*
 * Driver settings
 */
#define MEM2_NAME		"rvlmem2"
#define MEM2_MAJOR		Z2RAM_MAJOR

#define MEM2_SECTOR_SIZE	PAGE_SIZE


struct mem2_device {
	spinlock_t			lock;

	void __iomem			*io_base;

	struct block_device_operations	fops;
	struct gendisk			*disk;
	struct request_queue		*queue;

	int				ref_count;

	struct platform_device		pdev;	/* must be last member */
};


/* get the mem2 device given the platform device of a mem2 device */
#define to_mem2_device(n) container_of(n,struct mem2_device,pdev)

/*
 *
 */

/*
 * Performs block layer requests.
 */
static void mem2_do_request(struct request_queue *q)
{
	struct mem2_device *adev = q->queuedata;
	struct request *req;
	unsigned long mem2_addr;
	size_t len;
	int uptodate;

	req = elv_next_request(q);
	while(req) {
		if (blk_fs_request(req)) {
			/* calculate the MEM2 address and length */
			mem2_addr = req->sector << 9;
			len = req->current_nr_sectors << 9;

			/* give up if the request goes out of bounds */
			if (mem2_addr + len > MEM2_SIZE) {
				mem2_printk(KERN_ERR, "bad access: block=%lu,"
					    " size=%u\n",
					    (unsigned long)req->sector, len);
				uptodate = 0;
			} else {
				switch(rq_data_dir(req)) {
				case READ:
					memcpy(req->buffer,
					       adev->io_base + mem2_addr, len);
					break;
				case WRITE:
					memcpy(adev->io_base + mem2_addr,
					       req->buffer, len);
					break;
				}
				uptodate = 1;
			}
		} else {
			uptodate = 0;
		}
		end_queued_request(req, uptodate);
		req = elv_next_request(q);
	}
}

/*
 * Opens the MEM2 device.
 */
static int mem2_open(struct inode *inode, struct file *filp)
{
	struct mem2_device *adev = inode->i_bdev->bd_disk->private_data;
	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&adev->lock, flags);

	/* only allow a minor of 0 to be opened */
	if (iminor(inode)) {
		retval =  -ENODEV;
		goto out;
	}

	/* honor exclusive open mode */
	if (adev->ref_count == -1 ||
	    (adev->ref_count && (filp->f_flags & O_EXCL))) {
		retval = -EBUSY;
		goto out;
	}

	if ((filp->f_flags & O_EXCL))
		adev->ref_count = -1;
	else
		adev->ref_count++;

out:
	spin_unlock_irqrestore(&adev->lock, flags);
	return retval;
}

/*
 * Closes the MEM2 device.
 */
static int mem2_release(struct inode *inode, struct file *filp)
{
	struct mem2_device *adev = inode->i_bdev->bd_disk->private_data;
	unsigned long flags;

	spin_lock_irqsave(&adev->lock, flags);
	if (adev->ref_count > 0)
		adev->ref_count--;
	else
		adev->ref_count = 0;
	spin_unlock_irqrestore(&adev->lock, flags);
	
	return 0;
}

/*
 * Minimal ioctl for the MEM2 device.
 */
static int mem2_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)
{
	struct hd_geometry geo;
	
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
		geo.heads = 16;
		geo.sectors = 32;
		geo.start = 0;
		geo.cylinders = MEM2_SIZE / (geo.heads * geo.sectors);
		if (copy_to_user((void __user*)arg,&geo,sizeof(geo)))
			return -EFAULT;
		return 0;
	default:
		return -ENOTTY;
	}
}


static struct block_device_operations mem2_fops = {
	.owner = THIS_MODULE,
	.open = mem2_open,
	.release = mem2_release,
	.ioctl = mem2_ioctl,
};


/*
 *
 */
static int mem2_init_blk_dev(struct mem2_device *adev)
{
	struct gendisk *disk;
	struct request_queue *queue;
	int retval;

	adev->ref_count = 0;

	retval = register_blkdev(MEM2_MAJOR, MEM2_NAME);
	if (retval)
		goto err_register_blkdev;
	
	retval = -ENOMEM;
	spin_lock_init(&adev->lock);
	queue = blk_init_queue(mem2_do_request, &adev->lock);
	if (!queue)
		goto err_blk_init_queue;

	blk_queue_hardsect_size(queue, MEM2_SECTOR_SIZE);
	blk_queue_max_phys_segments(queue, 1);
	blk_queue_max_hw_segments(queue, 1);
	queue->queuedata = adev;
	adev->queue = queue;

	disk = alloc_disk(1);
	if (!disk)
		goto err_alloc_disk;

	disk->major = MEM2_MAJOR;
	disk->first_minor = 0;
	disk->fops = &mem2_fops;
	strcpy(disk->disk_name, MEM2_NAME);
	disk->queue = adev->queue;
	set_capacity(disk, MEM2_SIZE >> 9);
	disk->private_data = adev;
	adev->disk = disk;

	add_disk(adev->disk);

	retval = 0;
	goto out;

err_alloc_disk:
	blk_cleanup_queue(adev->queue);
err_blk_init_queue:
	unregister_blkdev(MEM2_MAJOR, MEM2_NAME);
err_register_blkdev:
out:
	return retval;
}

/*
 *
 */
static void mem2_exit_blk_dev(struct mem2_device *adev)
{
	if (adev->disk) {
		del_gendisk(adev->disk);
		put_disk(adev->disk);
	}
	if (adev->queue)
		blk_cleanup_queue(adev->queue);
	unregister_blkdev(MEM2_MAJOR, MEM2_NAME);
}

/*
 *
 */
static int mem2_init(struct mem2_device *adev, struct resource *mem)
{
	memset(adev, 0, sizeof(*adev) - sizeof(adev->pdev));

	adev->io_base = ioremap(MEM2_START, MEM2_SIZE);
	if (!adev->io_base) {
		mem2_printk(KERN_ERR, "unable to ioremap MEM2\n");
		return -EIO;
	}

	return mem2_init_blk_dev(adev);
}

/*
 *
 */
static void mem2_exit(struct mem2_device *adev)
{
	if (adev->io_base)
		iounmap(adev->io_base);
	mem2_exit_blk_dev(adev);
}

/*
 * Needed for platform devices.
 */
static void mem2_dev_release(struct device *dev)
{
}


static struct resource mem2_resources[] = {
	[0] = {
		.start = MEM2_START,
		.end = MEM2_END,
		.flags = IORESOURCE_MEM,
	},
};

static struct mem2_device mem2_device = {
	.pdev = {
	       	.name = MEM2_NAME,
	        .id = 0,
	        .num_resources = ARRAY_SIZE(mem2_resources),
	        .resource = mem2_resources,
		.dev = {
			.release = mem2_dev_release,
		},
	},
};


/*
 *
 */
static int mem2_probe(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mem2_device *adev = to_mem2_device(pdev);
	struct resource *mem;
	int retval;

	retval = -ENODEV;
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem) {
		retval = mem2_init(adev, mem);
	}
	return retval;
}

/*
 *
 */
static int mem2_remove(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct mem2_device *adev = to_mem2_device(pdev);

	mem2_exit(adev);

	return 0;
}


static struct device_driver mem2_driver = {
       	.name = MEM2_NAME,
	.bus = &platform_bus_type,
	.probe = mem2_probe,
	.remove = mem2_remove,
};

/*
 *
 */
static int __init mem2_init_module(void)
{
	int retval = 0;

	mem2_printk(KERN_INFO, "%s - version %s\n", DRV_DESCRIPTION,
		    mem2_driver_version);

	retval = driver_register(&mem2_driver);
	if (!retval) {
		retval = platform_device_register(&mem2_device.pdev);
	}

	return retval;
}

/*
 *
 */
static void __exit mem2_exit_module(void)
{
	platform_device_unregister(&mem2_device.pdev);
	driver_unregister(&mem2_driver);
}

module_init(mem2_init_module);
module_exit(mem2_exit_module);


MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_AUTHOR(DRV_AUTHOR);
MODULE_LICENSE("GPL");

