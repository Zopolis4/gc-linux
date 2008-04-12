/*
 * arch/powerpc/platforms/embedded6xx/starlet-ipc.c
 *
 * Nintendo Wii starlet IPC driver
 * Copyright (C) 2008 The GameCube Linux Team
 * Copyright (C) 2008 Albert Herranz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#define DEBUG

#define DBG(fmt, arg...)	pr_debug(fmt, ##arg)

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/string.h>
#include <asm/io.h>
#include <asm/bitops.h>

#include <asm/starlet.h>


#define DRV_MODULE_NAME		"starlet-ipc"
#define DRV_DESCRIPTION		"Nintendo Wii starlet IPC driver"
#define DRV_AUTHOR		"Albert Herranz"

static char starlet_ipc_driver_version[] = "0.2i";

#define drv_printk(level, format, arg...) \
        printk(level DRV_MODULE_NAME ": " format , ## arg)

/*
 * Hardware registers
 */
#define STARLET_IPC_TXBUF	0x00	/* data from cpu to starlet */

#define STARLET_IPC_CSR		0x04
#define   STARLET_IPC_CSR_TSTART	(1<<0)	/* start transmit */
#define   STARLET_IPC_CSR_TBEI		(1<<1)	/* tx buf empty int */
#define   STARLET_IPC_CSR_RBFI		(1<<2)	/* rx buf full int */
#define   STARLET_IPC_CSR_INT		(1<<3)	/* interrupt ack */
#define   STARLET_IPC_CSR_RBFIMASK	(1<<4)	/* rx buf full int mask */
#define   STARLET_IPC_CSR_TBEIMASK	(1<<5)	/* tx buf empty int mask */

#define STARLET_IPC_RXBUF	0x08	/* data from starlet to cpu */

#define STARLET_IPC_ISR		0x30

/* IOS calls */
#define STARLET_IOS_OPEN	0x01
#define STARLET_IOS_CLOSE	0x02
#define STARLET_IOS_IOCTL	0x06
#define STARLET_IOS_IOCTLV	0x07


/* starlet_ipc_device flags */
enum {
	__TX_INUSE = 0,		/* tx buffer in use flag */
	__REBOOT,		/* request causes IOS reboot */
};

/*
 * Misc.
 *
 */

/*
 * Simple aligned kzalloc and free.
 *
 * Based on the idea proposed by Satya Kiran Popuri
 * http://www.cs.uic.edu/~spopuri/amalloc.html
 */

/**
 *
 */
static void *kzalloc_aligned(size_t size, gfp_t flags, size_t align)
{
	void *ptr, *aligned_ptr;
	size_t aligned_size;

	/* not a power of two */
	if (align & (align - 1))
		return NULL;

	/* worst case allocation size */
	aligned_size = size + align - 1;

	/* add extra space to store allocation delta */
	aligned_size += sizeof(size_t);

	/* allocate all space */
	ptr = kzalloc(aligned_size, flags);
	if (!ptr)
		return NULL;

	/* calculate the aligned address, making room for the delta value */
	aligned_ptr = PTR_ALIGN(ptr + sizeof(size_t), align);

	/* save the delta before the address returned to caller */
	*((size_t *)aligned_ptr - 1) = aligned_ptr - ptr;

	return aligned_ptr;
}

static void kfree_aligned(void *aligned_ptr)
{
	void *ptr;
	size_t delta;

	if (!aligned_ptr)
		return;

	/* retrieve extra allocation delta */
	delta = *((size_t *)aligned_ptr - 1);

	/* calculate original allocation area start */
	ptr = aligned_ptr - delta;

	kfree(ptr);
}

/**
 *
 */
void *starlet_kzalloc(size_t size, gfp_t flags)
{
	return kzalloc_aligned(size, flags, STARLET_IPC_DMA_ALIGN+1);
}

/**
 *
 */
void starlet_kfree(void *ptr)
{
	kfree_aligned(ptr);
}


/*
 *
 * Hardware.
 */

/*
 * Update control and status register.
 */
static inline void starlet_ipc_update_csr(void __iomem * io_base, u32 val)
{
	u32 csr;

	csr = in_be32(io_base + STARLET_IPC_CSR);
	/* preserve interrupt masks */
	csr &= STARLET_IPC_CSR_RBFIMASK | STARLET_IPC_CSR_TBEIMASK;
	csr |= val;
	out_be32(io_base + STARLET_IPC_CSR, csr);
}

/*
 * Put data for starlet in the transmit fifo.
 */
static inline void starlet_ipc_sendto(void __iomem * io_base, u32 data)
{
	out_be32(io_base + STARLET_IPC_TXBUF, data);
}

/*
 * Get data from starlet out the receive fifo.
 */
static inline u32 starlet_ipc_recvfrom(void __iomem * io_base)
{
	return in_be32(io_base + STARLET_IPC_RXBUF);
}

/*
 * Issue an end-of-interrupt sequence.
 */
static void starlet_ipc_eoi(void __iomem * io_base)
{
	starlet_ipc_update_csr(io_base, STARLET_IPC_CSR_INT);
}

/*
 * Calm the hardware down.
 */
static void starlet_ipc_quiesce(struct starlet_ipc_device *ipc_dev)
{
	u32 csr;

	/* ack and disable MBOX? and REPLY interrupts */
	csr = in_be32(ipc_dev->io_base + STARLET_IPC_CSR);
	csr &= ~(STARLET_IPC_CSR_TBEIMASK | STARLET_IPC_CSR_RBFIMASK);
	csr |= STARLET_IPC_CSR_TBEI | STARLET_IPC_CSR_RBFI;
	out_be32(ipc_dev->io_base + STARLET_IPC_CSR, csr);
}

/*
 * Request routines.
 *
 */

static void starlet_ipc_debug_print_request(struct starlet_ipc_request *req)
{
#if 0
	DBG("cmd=%x, result=%x, fd=%x, dma_addr=%p\n",
	    req->cmd, req->result, req->fd, (void *)req->dma_addr);
#endif
}

static struct starlet_ipc_request *
starlet_ipc_alloc_request(struct starlet_ipc_device *ipc_dev)
{
	struct starlet_ipc_request *req;
	dma_addr_t dma_addr;

	req = dma_pool_alloc(ipc_dev->dma_pool, GFP_NOIO, &dma_addr);
	if (req) {
		memset(req, 0, sizeof(*req));
		req->ipc_dev = ipc_dev;
		req->result = 0xdeadbeef;
		req->dma_addr = dma_addr;
		INIT_LIST_HEAD(&req->node);
	}
	return req;
}

void starlet_ipc_free_request(struct starlet_ipc_request *req)
{
	dma_pool_free(req->ipc_dev->dma_pool, req, req->dma_addr);
}

static void starlet_ipc_start_request(struct starlet_ipc_request *req)
{
	struct starlet_ipc_device *ipc_dev = req->ipc_dev;
	void __iomem *io_base = ipc_dev->io_base;
	unsigned long flags;

	starlet_ipc_debug_print_request(req);

	spin_lock_irqsave(&ipc_dev->list_lock, flags);
	list_add_tail(&req->node, &ipc_dev->outstanding_list);
	ipc_dev->nr_outstanding++;
	spin_unlock_irqrestore(&ipc_dev->list_lock, flags);

	starlet_ipc_sendto(io_base, (u32) req->dma_addr);
	starlet_ipc_update_csr(io_base, STARLET_IPC_CSR_TSTART);
}

static void starlet_ipc_complete_request(struct starlet_ipc_request *req)
{
	struct starlet_ipc_device *ipc_dev = req->ipc_dev;
	unsigned long flags;

	spin_lock_irqsave(&ipc_dev->list_lock, flags);
	list_del(&req->node);
	ipc_dev->nr_outstanding--;
	spin_unlock_irqrestore(&ipc_dev->list_lock, flags);

	starlet_ipc_debug_print_request(req);

	if (req->done)
		req->done(req);
}

static void starlet_ipc_submit_request(struct starlet_ipc_request *req)
{
	struct starlet_ipc_device *ipc_dev = req->ipc_dev;
	unsigned long flags;

	if (test_and_set_bit(__TX_INUSE, &ipc_dev->flags)) {
		spin_lock_irqsave(&ipc_dev->list_lock, flags);
		list_add_tail(&req->node, &ipc_dev->pending_list);
		ipc_dev->nr_pending++;
		spin_unlock_irqrestore(&ipc_dev->list_lock, flags);
	} else {
		starlet_ipc_start_request(req);
	}
}

static struct starlet_ipc_request *
starlet_ipc_find_request_by_bus_addr(struct starlet_ipc_device *ipc_dev,
				     dma_addr_t	req_bus_addr)
{
	struct starlet_ipc_request *req;
	unsigned long flags;

	spin_lock_irqsave(&ipc_dev->list_lock, flags);
	list_for_each_entry(req, &ipc_dev->outstanding_list, node) {
		if (req && req_bus_addr == req->dma_addr) {
			spin_unlock_irqrestore(&ipc_dev->list_lock, flags);
			return req;
		}
	}
	spin_unlock_irqrestore(&ipc_dev->list_lock, flags);
	return NULL;
}

/*
 * Interrupt handlers.
 *
 */

/*
 * Transmit Buffer Empty Interrupt dispatcher.
 */
static int starlet_ipc_dispatch_tbei(struct starlet_ipc_device *ipc_dev)
{
	struct starlet_ipc_request *req = NULL;
	struct list_head *pending = &ipc_dev->pending_list;
	unsigned long flags;

	spin_lock_irqsave(&ipc_dev->list_lock, flags);
	if (!list_empty(pending)) {
		req = list_entry(pending->next, struct starlet_ipc_request,
				 node);
		list_del_init(&req->node);
		ipc_dev->nr_pending--;
	}
	spin_unlock_irqrestore(&ipc_dev->list_lock, flags);
	if (req) {
		starlet_ipc_start_request(req);
	} else {
		if (!test_and_clear_bit(__TX_INUSE, &ipc_dev->flags)) {
			/* we get two consecutive TBEIs on reboot */
			if (test_and_clear_bit(__REBOOT, &ipc_dev->flags)) {
				req = ipc_dev->req;
				ipc_dev->req = NULL;
				if (req) {
					starlet_ipc_complete_request(req);
					req->result = 0;
				}
			}
		}
	}

	return IRQ_HANDLED;
}

/*
 * Receive Buffer Full Interrupt dispatcher.
 */
static int starlet_ipc_dispatch_rbfi(struct starlet_ipc_device *ipc_dev)
{
	void __iomem *io_base = ipc_dev->io_base;
	struct starlet_ipc_request *req;
	unsigned long req_bus_addr;

	req_bus_addr = starlet_ipc_recvfrom(io_base);
	req = starlet_ipc_find_request_by_bus_addr(ipc_dev, req_bus_addr);
	if (req) {
		starlet_ipc_complete_request(req);
	} else {
		drv_printk(KERN_WARNING, "unknown request, bus=%p\n",
			   (void *)req_bus_addr);
	}

	return IRQ_HANDLED;
}

typedef int (*ipc_handler_t) (struct starlet_ipc_device *);

static int
starlet_ipc_cond_dispatch_irq(struct starlet_ipc_device *ipc_dev,
			      u32 irqmask, u32 irq, ipc_handler_t handler)
{
	void __iomem *io_base = ipc_dev->io_base;
	u32 csr;
	int retval = IRQ_NONE;

	csr = in_be32(io_base + STARLET_IPC_CSR);
	if ((csr & (irqmask | irq)) == (irqmask | irq)) {
		starlet_ipc_update_csr(io_base, irq);
		out_be32(io_base + STARLET_IPC_ISR, 0x40000000); /* huh? */
		retval = handler(ipc_dev);
	}
	return retval;
}

static irqreturn_t starlet_ipc_handler(int irq, void *data)
{
	struct starlet_ipc_device *ipc_dev = (struct starlet_ipc_device *)data;
	void __iomem *io_base = ipc_dev->io_base;
	int handled = 0;
	int retval;

	/* starlet read a request */
	retval = starlet_ipc_cond_dispatch_irq(ipc_dev,
					       STARLET_IPC_CSR_TBEIMASK,
					       STARLET_IPC_CSR_TBEI,
					       starlet_ipc_dispatch_tbei);
	if (retval == IRQ_HANDLED)
		handled++;

	/* starlet wrote a reply */
	retval = starlet_ipc_cond_dispatch_irq(ipc_dev,
					       STARLET_IPC_CSR_RBFIMASK,
					       STARLET_IPC_CSR_RBFI,
					       starlet_ipc_dispatch_rbfi);
	if (retval == IRQ_HANDLED)
		handled++;

	if (!handled)
		return IRQ_NONE;

	starlet_ipc_eoi(io_base);
	return IRQ_HANDLED;
}

/*
 * IPC Calls.
 *
 */

static int starlet_ipc_call_done(struct starlet_ipc_request *req)
{
	complete(req->done_data);
	return 0;
}

static int starlet_ipc_call(struct starlet_ipc_request *req)
{
	DECLARE_COMPLETION(complete);

	req->done_data = &complete;
	req->done = starlet_ipc_call_done;
	starlet_ipc_submit_request(req);
	wait_for_completion(&complete);
	return req->result;
}

static void starlet_ipc_call_nowait(struct starlet_ipc_request *req,
				    starlet_ipc_callback_t callback, void *arg)
{
	req->done_data = arg;
	req->done = callback;
	starlet_ipc_submit_request(req);
}

/*
 *
 * IOS High level interfaces.
 */

static struct starlet_ipc_device *starlet_ipc_device_instance;

/**
 *
 */
struct starlet_ipc_device *starlet_ipc_get_device(void)
{
	if (!starlet_ipc_device_instance)
		drv_printk(KERN_ERR, "uninitialized device instance!\n");
	return starlet_ipc_device_instance;
}
EXPORT_SYMBOL_GPL(starlet_ipc_get_device);

/**
 *
 */
int starlet_ios_open(const char *pathname, int flags)
{
#define STSD_OPEN_BUF_SIZE	64
	static char open_buf[STSD_OPEN_BUF_SIZE]
		    __attribute__ ((aligned(STARLET_IPC_DMA_ALIGN + 1)));
	static DEFINE_MUTEX(open_buf_lock);

	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	struct starlet_ipc_request *req;
	dma_addr_t dma_addr;
	char *local_pathname = NULL;
	size_t len;
	int error = -ENOMEM;

	if (!ipc_dev)
		return -ENODEV;

	req = starlet_ipc_alloc_request(ipc_dev);
	if (req) {
		len = strlen(pathname) + 1;
		if (len < sizeof(open_buf)) {
			if (mutex_trylock(&open_buf_lock))
				local_pathname = open_buf;
		}
		if (!local_pathname) {
			local_pathname = starlet_kzalloc(len, GFP_KERNEL);
			if (!local_pathname) {
				starlet_ipc_free_request(req);
				return -ENOMEM;
			}
		}

		strncpy(local_pathname, pathname, len-1);
		local_pathname[len-1] = 0;
		dma_addr = dma_map_single(ipc_dev->dev, local_pathname, len,
					  DMA_TO_DEVICE);

		req->cmd = STARLET_IOS_OPEN;
		req->open.pathname = dma_addr;	/* bus address */
		req->open.mode = flags;
		error = starlet_ipc_call(req);

		dma_unmap_single(ipc_dev->dev, dma_addr, len, DMA_TO_DEVICE);

		if (local_pathname == open_buf)
			mutex_unlock(&open_buf_lock);
		else
			starlet_kfree(local_pathname);

		starlet_ipc_free_request(req);
	}
	if (error < 0)
		DBG("%s: error=%d (%x)\n", __func__, error, error);
	return error;
}
EXPORT_SYMBOL_GPL(starlet_ios_open);

/**
 *
 */
int starlet_ios_close(int fd)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	struct starlet_ipc_request *req;
	int error = -ENOMEM;

	if (!ipc_dev)
		return -ENODEV;

	req = starlet_ipc_alloc_request(ipc_dev);
	if (req) {
		req->cmd = STARLET_IOS_CLOSE;
		req->fd = fd;
		error = starlet_ipc_call(req);
		starlet_ipc_free_request(req);
	}
	return error;
}
EXPORT_SYMBOL_GPL(starlet_ios_close);


/*
 * starlet_ios_ioctl*
 *
 */

static struct starlet_ipc_request *
starlet_ios_ioctl_dma_prepare(int fd, int request,
			      dma_addr_t ibuf, size_t ilen,
			      dma_addr_t obuf, size_t olen)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	struct starlet_ipc_request *req;

	if (!ipc_dev)
		return NULL;

	req = starlet_ipc_alloc_request(ipc_dev);
	if (req) {
		req->cmd = STARLET_IOS_IOCTL;
		req->fd = fd;
		req->ioctl.request = (u32) request;
		req->ioctl.ibuf = ibuf;
		req->ioctl.ilen = ilen;
		req->ioctl.obuf = obuf;
		req->ioctl.olen = olen;

#if 0
		DBG("%s: fd=%d, request=%x,"
		    " ibuf=%x, ilen=%u, obuf=%x, olen=%u\n" ,  __func__,
				req->fd, req->ioctl.request,
				req->ioctl.ibuf, req->ioctl.ilen,
				req->ioctl.obuf, req->ioctl.olen);
#endif
	}
	return req;

}

/**
 *
 */
static void starlet_ios_ioctl_dma_complete(struct starlet_ipc_request *req)
{
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctl_dma_complete);

/**
 *
 */
int starlet_ios_ioctl_dma(int fd, int request,
			  dma_addr_t ibuf, size_t ilen,
			  dma_addr_t obuf, size_t olen)
{
	struct starlet_ipc_request *req;
	int error = -ENOMEM;

	req = starlet_ios_ioctl_dma_prepare(fd, request, ibuf, ilen, obuf, olen);
	if (req) {
		error = starlet_ipc_call(req);
		starlet_ios_ioctl_dma_complete(req);
		starlet_ipc_free_request(req);
	}
	if (error)
		DBG("%s: error=%d (%x)\n", __func__, error, error);
	return error;
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctl_dma);

/**
 *
 */
int starlet_ios_ioctl_dma_nowait(int fd, int request,
				 dma_addr_t ibuf, size_t ilen,
				 dma_addr_t obuf, size_t olen,
				 starlet_ipc_callback_t callback, void *arg)
{
	struct starlet_ipc_request *req;
	int error = -ENOMEM;

	req = starlet_ios_ioctl_dma_prepare(fd, request, ibuf, ilen, obuf, olen);
	if (req) {
		starlet_ipc_call_nowait(req, callback, arg);
		error = 0;
	}
	return error;
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctl_dma_nowait);

static struct starlet_ipc_request *
starlet_ios_ioctl_prepare(int fd, int request,
			void *ibuf, size_t ilen,
			void *obuf, size_t olen)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	struct starlet_ipc_request *req;
	dma_addr_t ibuf_ba, obuf_ba;

	if (!ipc_dev)
		return NULL;

	BUG_ON(!IS_ALIGNED((unsigned long)ibuf, STARLET_IPC_DMA_ALIGN+1));
	BUG_ON(!IS_ALIGNED((unsigned long)obuf, STARLET_IPC_DMA_ALIGN+1));

	ibuf_ba = (ibuf) ? dma_map_single(ipc_dev->dev, ibuf, ilen,
					  DMA_TO_DEVICE) : 0;
	obuf_ba = (obuf) ? dma_map_single(ipc_dev->dev, obuf, olen,
					  DMA_FROM_DEVICE) : 0;

	req = starlet_ios_ioctl_dma_prepare(fd, request,
					  ibuf_ba, ilen, obuf_ba, olen);
	if (!req) {
		if (ibuf)
			dma_unmap_single(ipc_dev->dev, ibuf_ba, ilen,
					 DMA_TO_DEVICE);
		if (obuf)
			dma_unmap_single(ipc_dev->dev, obuf_ba, olen,
					 DMA_FROM_DEVICE);
	}
	return req;
}

/**
 *
 */
void starlet_ios_ioctl_complete(struct starlet_ipc_request *req)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	dma_addr_t ibuf_ba, obuf_ba;
	size_t ilen, olen;

	ibuf_ba = req->ioctl.ibuf;
	ilen = req->ioctl.ilen;
	obuf_ba = req->ioctl.obuf;
	olen = req->ioctl.olen;

	if (ibuf_ba)
		dma_unmap_single(ipc_dev->dev, ibuf_ba, ilen, DMA_TO_DEVICE);
	if (obuf_ba)
		dma_unmap_single(ipc_dev->dev, obuf_ba, olen, DMA_FROM_DEVICE);
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctl_complete);

/**
 *
 */
int starlet_ios_ioctl(int fd, int request,
		      void *ibuf, size_t ilen,
		      void *obuf, size_t olen)
{
	struct starlet_ipc_request *req;
	int error = -ENOMEM;

	req = starlet_ios_ioctl_prepare(fd, request, ibuf, ilen, obuf, olen);
	if (req) {
		error = starlet_ipc_call(req);
		starlet_ios_ioctl_complete(req);
		starlet_ipc_free_request(req);
	}
	if (error < 0)
		DBG("%s: error=%d (%x)\n", __func__, error, error);
	return error;
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctl);

/**
 *
 */
int starlet_ios_ioctl_nowait(int fd, int request,
			     void *ibuf, size_t ilen,
			     void *obuf, size_t olen,
			     starlet_ipc_callback_t callback, void *arg)
{
	struct starlet_ipc_request *req;
	int error = -ENOMEM;

	req = starlet_ios_ioctl_prepare(fd, request, ibuf, ilen, obuf, olen);
	if (req) {
		starlet_ipc_call_nowait(req, callback, arg);
		error = 0;
	}
	return error;
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctl_nowait);

/*
 * starlet_ios_ioctlv*
 *
 */

static struct starlet_ipc_request *
starlet_ios_ioctlv_prepare(int fd, int request,
			   unsigned int nents_in,
			   struct scatterlist *sgl_in,
			   unsigned int nents_out,
			   struct scatterlist *sgl_out)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	struct starlet_ipc_request *req;
	struct starlet_iovec *iovec, *p;
	dma_addr_t iovec_da = 0;
	size_t iovec_size = 0;
	struct scatterlist *sg;
	unsigned int nents, i;

	if (!ipc_dev)
		return NULL;

	BUG_ON(nents_in > 0 && !sgl_in);
	BUG_ON(nents_out > 0 && !sgl_out);

	nents  = nents_in + nents_out;
	if (nents > 0) {
		iovec_size = nents * sizeof(*iovec);
		iovec = starlet_kzalloc(iovec_size, GFP_NOIO);
		if (!iovec)
			return NULL;
	} else {
		iovec = NULL;
	}

	req = starlet_ipc_alloc_request(ipc_dev);
	if (req) {
		p = iovec;
		if (nents_in > 0) {
			nents_in = dma_map_sg(ipc_dev->dev, sgl_in, nents_in,
					      DMA_TO_DEVICE);
			for_each_sg(sgl_in, sg, nents_in, i) {
				p->dma_addr = sg_dma_address(sg);
				p->dma_len = sg_dma_len(sg);
				p++;
			}
		}
		if (nents_out > 0) {
			nents_out = dma_map_sg(ipc_dev->dev, sgl_out, nents_out,
					       DMA_FROM_DEVICE);
			for_each_sg(sgl_out, sg, nents_out, i) {
				p->dma_addr = sg_dma_address(sg);
				p->dma_len = sg_dma_len(sg);
				p++;
			}
		}

		if (iovec)
			iovec_da = dma_map_single(ipc_dev->dev,
						  iovec, iovec_size,
						  DMA_TO_DEVICE);

		req->iovec = iovec;
		req->iovec_size = iovec_size;
		req->sgl_nents_in = nents_in;
		req->sgl_in = sgl_in;
		req->sgl_nents_out = nents_out;
		req->sgl_out = sgl_out;

		req->cmd = STARLET_IOS_IOCTLV;
		req->fd = fd;
		req->ioctlv.request = request;
		req->ioctlv.argc_in = nents_in;
		req->ioctlv.argc_out = nents_out;
		req->ioctlv.iovec_da = iovec_da;

#if 0
		DBG("%s: fd=%d, request=%d,"
		    " argc_in=%u, argc_out=%u, iovec_da=%08x\n" ,  __func__,
				req->fd, req->ioctlv.request,
				req->ioctlv.argc_in, req->ioctlv.argc_out,
				req->ioctlv.iovec_da);
#endif
	} else {
		if (iovec)
			starlet_kfree(iovec);
	}
	return req;
}

/**
 *
 */
void starlet_ios_ioctlv_complete(struct starlet_ipc_request *req)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	struct starlet_iovec *iovec = req->iovec;
	dma_addr_t iovec_da = req->ioctlv.iovec_da;
	size_t iovec_size = req->iovec_size;

	if (req->sgl_nents_in > 0)
		dma_unmap_sg(ipc_dev->dev, req->sgl_in, req->sgl_nents_in,
			     DMA_TO_DEVICE);
	if (req->sgl_nents_out > 0)
		dma_unmap_sg(ipc_dev->dev, req->sgl_out, req->sgl_nents_out,
			     DMA_FROM_DEVICE);
	if (iovec) {
		dma_unmap_single(ipc_dev->dev,
				 iovec_da, iovec_size, DMA_TO_DEVICE);
		starlet_kfree(iovec);
	}
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctlv_complete);

/**
 *
 */
int starlet_ios_ioctlv(int fd, int request,
		       unsigned int nents_in,
		       struct scatterlist *sgl_in,
		       unsigned int nents_out,
		       struct scatterlist *sgl_out)
{
	struct starlet_ipc_request *req;
	int error = -ENOMEM;

	req = starlet_ios_ioctlv_prepare(fd, request,
					 nents_in, sgl_in,
					 nents_out, sgl_out);
	if (req) {
		error = starlet_ipc_call(req);
		starlet_ios_ioctlv_complete(req);
		starlet_ipc_free_request(req);
	}
	if (error < 0)
		DBG("%s: error=%d (%x)\n", __func__, error, error);
	return error;
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctlv);

/**
 *
 */
int starlet_ios_ioctlv_nowait(int fd, int request,
			      unsigned int nents_in,
			      struct scatterlist *sgl_in,
			      unsigned int nents_out,
			      struct scatterlist *sgl_out,
			      starlet_ipc_callback_t callback, void *arg)
{
	struct starlet_ipc_request *req;
	int error = -ENOMEM;

	req = starlet_ios_ioctlv_prepare(fd, request,
					 nents_in, sgl_in,
					 nents_out, sgl_out);
	if (req) {
		starlet_ipc_call_nowait(req, callback, arg);
		error = 0;
	}
	return error;
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctlv_nowait);

/**
 *
 */
int starlet_ios_ioctlv_and_reboot(int fd, int request,
				  unsigned int nents_in,
				  struct scatterlist *sgl_in,
				  unsigned int nents_out,
				  struct scatterlist *sgl_out)
{
	struct starlet_ipc_device *ipc_dev;
	struct starlet_ipc_request *req;
	int error = -ENOMEM;

	req = starlet_ios_ioctlv_prepare(fd, request,
					 nents_in, sgl_in,
					 nents_out, sgl_out);
	if (req) {
		ipc_dev = req->ipc_dev;
		ipc_dev->req = req;
		set_bit(__REBOOT, &ipc_dev->flags);
		error = starlet_ipc_call(req);
		starlet_ios_ioctlv_complete(req);
		starlet_ipc_free_request(req);
	}
	if (error < 0)
		DBG("%s: error=%d (%x)\n", __func__, error, error);
	return error;
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctlv_and_reboot);



/*
 * Setup routines.
 *
 */

/*
 * Place here any desired hardware cleanups while drivers get written.
 */
static void starlet_ios_fixups(void)
{
	static u32 buf[8]
	    __attribute__ ((aligned(STARLET_IPC_DMA_ALIGN + 1)));
	struct scatterlist in[6], out[1];
	int fd;
	void *gpio;

	/* close any open file descriptors, just in case */
	for(fd = 0; fd < 15; fd++)
		starlet_ios_close(fd);

	/*
	 * Hey! We are super-green. And you?
	 */

	/* try to stop the dvd unit motor */
	fd = starlet_ios_open("/dev/di", 0);
	if (fd >= 0) {
		buf[0] = 0xe3000000; /* stop motor command */
		buf[1] = 0;
		buf[2] = 0;
		starlet_ios_ioctl(fd, buf[0],
				  buf, sizeof(buf),
				  buf, sizeof(buf));
		starlet_ios_close(fd);
	}

	/* try to disconnect the wiimote */
	fd = starlet_ios_open("/dev/usb/oh1/57e/305", 2);
	if (fd >= 0) {
		/*
		 * This assumes big endianness and 4 byte dma alignment.
		 */
		buf[0] = 0x20000000; /* bmRequestType	0x20 */
		buf[1] = 0x00000000; /* bmRequestType	0x00 */
		buf[2] = 0x00000000; /* wValue		0x00, 0x00 */
		buf[3] = 0x00000000; /* wIndex		0x00, 0x00 */
		buf[4] = 0x03000000; /* wLength		0x03, 0x00 */
		buf[5] = 0x00000000; /* unknown?	0x00 */
		buf[6] = 0x030c0000; /* payload?	0x03, 0x0c, 0x00 */
		sg_init_table(in, 6);
		sg_set_buf(&in[0], &buf[0], 1);
		sg_set_buf(&in[1], &buf[1], 1);
		sg_set_buf(&in[2], &buf[2], 2);
		sg_set_buf(&in[3], &buf[3], 2);
		sg_set_buf(&in[4], &buf[4], 2);
		sg_set_buf(&in[5], &buf[5], 1);
		sg_init_table(out, 1);
		sg_set_buf(&out[0], &buf[6], 3);
		starlet_ios_ioctlv(fd, 0, 6, in, 1, out);
		starlet_ios_close(fd);
	}

	/* try to turn off the front led (not starlet-related but anyway...) */
	gpio = ioremap(0x0d8000c0, 4);
	if (gpio) {
		out_be32(gpio, in_be32(gpio) & ~0x20);
		iounmap(gpio);
	}
}

static int starlet_ipc_init(struct starlet_ipc_device *ipc_dev,
			    struct resource *mem, int irq)
{
	size_t size;
	int retval;

	ipc_dev->io_base = ioremap(mem->start, mem->end - mem->start + 1);
	ipc_dev->irq = irq;

	size = max((size_t)64, sizeof(struct starlet_ipc_request));
	ipc_dev->dma_pool = dma_pool_create(DRV_MODULE_NAME,
					    ipc_dev->dev,
					    size, STARLET_IPC_DMA_ALIGN + 1, 0);
	if (!ipc_dev->dma_pool) {
		drv_printk(KERN_ERR, "dma_pool_create failed\n");
		iounmap(ipc_dev->io_base);
		return -ENOMEM;
	}
	spin_lock_init(&ipc_dev->list_lock);
	INIT_LIST_HEAD(&ipc_dev->pending_list);
	INIT_LIST_HEAD(&ipc_dev->outstanding_list);

	starlet_ipc_device_instance = ipc_dev;

	retval = request_irq(ipc_dev->irq, starlet_ipc_handler, 0,
			     DRV_MODULE_NAME, ipc_dev);
	if (retval) {
		drv_printk(KERN_ERR, "request of IRQ %d failed\n", irq);
		starlet_ipc_device_instance = NULL;
		dma_pool_destroy(ipc_dev->dma_pool);
		iounmap(ipc_dev->io_base);
		return retval;
	}

	/* ack and enable RBFI and TBEI interrupts */
	out_be32(ipc_dev->io_base + STARLET_IPC_CSR,
		 STARLET_IPC_CSR_TBEIMASK | STARLET_IPC_CSR_RBFIMASK |
		 STARLET_IPC_CSR_TBEI | STARLET_IPC_CSR_RBFI);

	starlet_ios_fixups();

	return retval;
}

static void starlet_ipc_exit(struct starlet_ipc_device *ipc_dev)
{
	starlet_ipc_device_instance = NULL;
	starlet_ipc_quiesce(ipc_dev);

	free_irq(ipc_dev->irq, ipc_dev);
	dma_pool_destroy(ipc_dev->dma_pool);
	iounmap(ipc_dev->io_base);
	ipc_dev->io_base = NULL;
}


/*
 * Driver model helper routines.
 *
 */

static int starlet_ipc_do_probe(struct device *dev, struct resource *mem,
				int irq)
{
	struct starlet_ipc_device *ipc_dev;
	int retval;

	ipc_dev = kzalloc(sizeof(*ipc_dev), GFP_KERNEL);
	if (!ipc_dev) {
		drv_printk(KERN_ERR, "failed to allocate ipc_dev\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev, ipc_dev);
	ipc_dev->dev = dev;

	retval = starlet_ipc_init(ipc_dev, mem, irq);
	if (retval) {
		dev_set_drvdata(dev, NULL);
		kfree(ipc_dev);
	}
	return retval;
}

static int starlet_ipc_do_remove(struct device *dev)
{
	struct starlet_ipc_device *ipc_dev = dev_get_drvdata(dev);

	if (ipc_dev) {
		starlet_ipc_exit(ipc_dev);
		dev_set_drvdata(dev, NULL);
		kfree(ipc_dev);
		return 0;
	}
	return -ENODEV;
}

static int starlet_ipc_do_shutdown(struct device *dev)
{
	struct starlet_ipc_device *ipc_dev = dev_get_drvdata(dev);

	if (ipc_dev) {
		starlet_ipc_quiesce(ipc_dev);
		return 0;
	}
	return -ENODEV;
}

/*
 * OF platform driver hooks.
 *
 */

static int starlet_ipc_of_probe(struct of_device *odev,
				const struct of_device_id *dev_id)
{
	struct resource res;
	int retval;

	retval = of_address_to_resource(odev->node, 0, &res);
	if (retval) {
		drv_printk(KERN_ERR, "no io memory range found\n");
		return -ENODEV;
	}

	return starlet_ipc_do_probe(&odev->dev, &res,
				    irq_of_parse_and_map(odev->node, 0));
}

static int starlet_ipc_of_remove(struct of_device *odev)
{
	return starlet_ipc_do_remove(&odev->dev);
}

static int starlet_ipc_of_shutdown(struct of_device *odev)
{
	return starlet_ipc_do_shutdown(&odev->dev);
}

static struct of_device_id starlet_ipc_of_match[] = {
	{ .compatible = "nintendo,starlet-ipc" },
	{ },
};

MODULE_DEVICE_TABLE(of, starlet_ipc_of_match);

static struct of_platform_driver starlet_ipc_of_driver = {
	.owner = THIS_MODULE,
	.name = DRV_MODULE_NAME,
	.match_table = starlet_ipc_of_match,
	.probe = starlet_ipc_of_probe,
	.remove = starlet_ipc_of_remove,
	.shutdown = starlet_ipc_of_shutdown,
};

/*
 * Kernel module interface hooks.
 *
 */

static int __init starlet_ipc_init_module(void)
{
	drv_printk(KERN_INFO, "%s - version %s\n", DRV_DESCRIPTION,
		   starlet_ipc_driver_version);

	return of_register_platform_driver(&starlet_ipc_of_driver);
}

static void __exit starlet_ipc_exit_module(void)
{
	of_unregister_platform_driver(&starlet_ipc_of_driver);
}

module_init(starlet_ipc_init_module);
module_exit(starlet_ipc_exit_module);

MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_AUTHOR(DRV_AUTHOR);
MODULE_LICENSE("GPL");

