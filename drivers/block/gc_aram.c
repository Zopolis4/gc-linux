/*
	gc_aram.c
	
	by GC-Linux Team , hamtitampti , 2004
*/
/*
 	// Some test things
 	
 	echo hello1234567890hello1234567890 | dd of=/dev/aram seek=10
 	dd if=/dev/aram skip=10
  	cat /dev/aram  | wc -c

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

#define ARAM_MAJOR Z2RAM_MAJOR


#define TRUE                  (1)
#define FALSE                 (0)

//#define ARAM_DBG	printk
#define ARAM_DBG(format, arg...); { }

//#define RAMDISK
#undef RAMDISK

#ifdef RAMDISK
	unsigned char *RAMDISKBuffer;
	#define ARAM_BUFFERSIZE 10*1024
#else
	#define ARAM_BUFFERSIZE 14*1024*1024
	//#define ARAM_BUFFERSIZE 10*1024
	#define ARAM_SOUNDMEMORYOFFSET	1024*1024
#endif





#define ARAM_CHUNKMASK		0xff


static int current_device   = -1;
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
#define AI_DSP_STATUS	 		*(unsigned short*)0xCC00500A

#define ARAM_READ				1
#define ARAM_WRITE				0

int ARAM_DMA_lock = 0;

void ARAM_StartDMA (unsigned long mmAddr, unsigned long arAddr, unsigned long length, unsigned long type)
{
	while(ARAM_DMA_lock);
	ARAM_DMA_lock = 1;
 	
	//printk("ARAM DMA copy -> %08x - %08x  - %d  %d\n",mmAddr,arAddr,length,type);
	
	AR_DMA_MMADDR_H = mmAddr >> 16;
	AR_DMA_MMADDR_L = mmAddr & 0xFFFF;
	
	AR_DMA_ARADDR_H = arAddr >> 16;
	AR_DMA_ARADDR_L = arAddr & 0xFFFF;
	
	AR_DMA_CNT_H = (type << 15) | (length >> 16);
	AR_DMA_CNT_L = length & 0xFFFF;
	
	// For security
	udelay(10000);

	// Without the Break, the While loop loops endless
	int counter=0;
	while (AI_DSP_STATUS & 0x200) {
		counter++;
		if (counter>0xfffff) break;
	};
	ARAM_DMA_lock = 0;

}
/*
 	echo YUHUUhello1234567890hello12345678901234567890CCC > /dev/aram 
 	dd if=/dev/aram 
 	cat /dev/aram  | wc -c
*/
static void do_aram_request(request_queue_t *q)
{
	struct request *req;
	while ((req = elv_next_request(q)) != NULL) {
		unsigned long start = req->sector << 9;
		unsigned long len  = req->current_nr_sectors << 9;

		if (start + len > ARAM_BUFFERSIZE) {
			printk( KERN_ERR DEVICE_NAME ": bad access: block=%lu, count=%u\n",
				req->sector, req->current_nr_sectors);
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
			flush_dcache_range((unsigned long)req->buffer,(unsigned long)req->buffer + len);
			ARAM_StartDMA((unsigned long)req->buffer,start+ARAM_SOUNDMEMORYOFFSET, len,ARAM_READ);
			flush_dcache_range((unsigned long)req->buffer,(unsigned long)req->buffer + len);
		} else {
			flush_dcache_range((unsigned long)req->buffer,(unsigned long)req->buffer + len);
			ARAM_StartDMA((unsigned long)req->buffer,start+ARAM_SOUNDMEMORYOFFSET, len,ARAM_WRITE);
		}			
		#endif		
		
		end_request(req, 1);
	}
}


static int aram_open( struct inode *inode, struct file *filp )
{
	ARAM_DBG("A-RAM Open device\n");
	
	int device;
	int rc = -ENOMEM;
	
	device = iminor(inode);
	
	if ( current_device != -1 && current_device != device )
	{
		rc = -EBUSY;
		goto err_out;
	}
    
	#ifdef RAMDISK
	if ( current_device == -1 )
	{
		current_device = device;
		set_capacity(aram_gendisk,ARAM_BUFFERSIZE>>9);
	}
	#endif
	return 0;
	
err_out:
  	ARAM_DBG("A-RAM Open device Error %d\n",rc);
    	
    	return rc;

}

static int aram_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	ARAM_DBG("A-RAM IOCTL\n");
	return 0;
}

static int aram_release( struct inode *inode, struct file *filp )
{
	ARAM_DBG("A-RAM Close device\n");
    	if ( current_device == -1 ) return 0;     

	return 0;
}

static struct block_device_operations aram_fops =
{
	.owner		= THIS_MODULE,
	.open		= aram_open,
	.release	= aram_release,
	.ioctl 		= aram_ioctl,
};


static struct request_queue *aram_queue;

int __init aram_init(void)
{
	int ret;
	
	printk("A-Ram Block Device Driver Init\n");
	
	ret = -EBUSY;
	if (register_blkdev(ARAM_MAJOR, DEVICE_NAME)) goto err;
	
	ret = -ENOMEM;
	aram_gendisk = alloc_disk(1);
	if (!aram_gendisk) goto out_disk;
	
	aram_queue = blk_init_queue(do_aram_request, &aram_lock);
	if (!aram_queue) goto out_queue;
	
	aram_gendisk->major = ARAM_MAJOR;
	aram_gendisk->first_minor = 0;
	aram_gendisk->fops = &aram_fops;
	sprintf(aram_gendisk->disk_name, "aram");
	strcpy(aram_gendisk->devfs_name, aram_gendisk->disk_name);
	
	aram_gendisk->queue = aram_queue;
	set_capacity(aram_gendisk,ARAM_BUFFERSIZE>>9);
	add_disk(aram_gendisk);
	
	#ifdef RAMDISK
	RAMDISKBuffer = kmalloc(ARAM_BUFFERSIZE,GFP_KERNEL);
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
	if ( unregister_blkdev( ARAM_MAJOR, DEVICE_NAME ) != 0 )
		printk( KERN_ERR DEVICE_NAME ": unregister of device failed\n");
	
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

