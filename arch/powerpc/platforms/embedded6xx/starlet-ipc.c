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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <asm/io.h>
#include <asm/bitops.h>

#include <asm/starlet.h>

#ifdef CONFIG_PPC_MERGE
#include <platforms/embedded6xx/gamecube.h>
#else
#include <platforms/gamecube.h>
#endif


#define DRV_MODULE_NAME		"starlet-ipc"
#define DRV_DESCRIPTION		"Nintendo Wii starlet IPC driver"
#define DRV_AUTHOR		"Albert Herranz"

static char starlet_ipc_driver_version[] = "0.1-isobel";


#define PFX DRV_MODULE_NAME ": "
#define ipc_printk(level, format, arg...) \
        printk(level PFX format , ## arg)

#ifdef SD_DEBUG
#  define DBG(fmt, args...) \
          printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DBG(fmt, args...)
#endif


#define STARLET_IPC_BASE	0x0d000000
#define STARLET_IPC_SIZE	0x40

#define STARLET_IPC_IRQ		14

#define STARLET_IPC_DMA_ALIGN	0x1f	/* 32 bytes */

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


/* starlet_ipc_device flags */
enum {
	__TX_INUSE = 0,		/* tx buffer in use flag */
};


struct starlet_ipc_request {
	/* begin starlet hardware request format */
	u32 cmd;
	s32 result;
	union {
		s32 fd;
		u32 req_cmd;
	};
	union {
		struct {
			dma_addr_t pathname;
			u32 mode;
		} open;
		struct {
			u32 request;
			dma_addr_t ibuf;
			u32 ilen;
			dma_addr_t obuf;
			u32 olen;
		} ioctl;
		u32 argv[5];
	};
	/* end starlet hardware request format */

	struct starlet_ipc_device *ipc_dev;
	struct list_head node;

	dma_addr_t dma_addr;

	void *done_data;
	void (*done) (struct starlet_ipc_request * req);
};


/*
 *
 * Hardware.
 */

/*
 *
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
 *
 */
static inline void starlet_ipc_sendto(void __iomem * io_base, u32 data)
{
	out_be32(io_base + STARLET_IPC_TXBUF, data);
}

/*
 *
 */
static inline u32 starlet_ipc_recvfrom(void __iomem * io_base)
{
	return in_be32(io_base + STARLET_IPC_RXBUF);
}

/*
 *
 */
static void starlet_ipc_eoi(void __iomem * io_base)
{
	starlet_ipc_update_csr(io_base, STARLET_IPC_CSR_INT);
	out_be32(io_base + STARLET_IPC_ISR, 0x40000000);	/* huh? */
}

/*
 *
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
 * Requests.
 *
 */

/*
 *
 */
static void starlet_ipc_debug_print_request(struct starlet_ipc_request *req)
{
	DBG("cmd=%x, result=%d, fd=%x, dma_addr=%p\n",
	    req->cmd, req->result, req->fd, (void *)req->dma_addr);
}

/*
 *
 */
static struct starlet_ipc_request *starlet_ipc_alloc_request(struct
							     starlet_ipc_device
							     *ipc_dev)
{
	struct starlet_ipc_request *req;
	dma_addr_t dma_addr;

	req = dma_pool_alloc(ipc_dev->dma_pool, GFP_KERNEL, &dma_addr);
	if (req) {
		req->ipc_dev = ipc_dev;
		req->result = 0xdeadbeef;
		req->dma_addr = dma_addr;
		INIT_LIST_HEAD(&req->node);
	}
	return req;
}

/*
 *
 */
void starlet_ipc_free_request(struct starlet_ipc_request *req)
{
	dma_pool_free(req->ipc_dev->dma_pool, req, req->dma_addr);
}

/*
 *
 */
static void starlet_ipc_start_request(struct starlet_ipc_request *req)
{
	struct starlet_ipc_device *ipc_dev = req->ipc_dev;
	void __iomem *io_base = ipc_dev->io_base;
	unsigned long flags;

	DBG("start_request\n");
	starlet_ipc_debug_print_request(req);

	spin_lock_irqsave(&ipc_dev->list_lock, flags);
	list_add_tail(&req->node, &ipc_dev->outstanding_list);
	ipc_dev->nr_outstanding++;
	spin_unlock_irqrestore(&ipc_dev->list_lock, flags);

	starlet_ipc_sendto(io_base, (u32) req->dma_addr);
	starlet_ipc_update_csr(io_base, STARLET_IPC_CSR_TSTART);
}

/*
 *
 */
static void starlet_ipc_complete_request(struct starlet_ipc_request *req)
{
	struct starlet_ipc_device *ipc_dev = req->ipc_dev;
	unsigned long flags;

	spin_lock_irqsave(&ipc_dev->list_lock, flags);
	list_del(&req->node);
	ipc_dev->nr_outstanding--;
	spin_unlock_irqrestore(&ipc_dev->list_lock, flags);

	DBG("complete_request\n");
	starlet_ipc_debug_print_request(req);

	if (req->done)
		req->done(req);
}

/*
 *
 */
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

/*
 *
 */
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
 * Transmit Buffer Empty Interrupt.
 */
static int starlet_ipc_dispatch_tbei(struct starlet_ipc_device *ipc_dev)
{
	struct starlet_ipc_request *req = NULL;
	struct list_head *pending = &ipc_dev->pending_list;
	unsigned long flags;

	DBG("tx buf empty interrupt!\n");

	spin_lock_irqsave(&ipc_dev->list_lock, flags);
	if (!list_empty(pending)) {
		req = list_entry(pending->next, struct starlet_ipc_request,
				 node);
		list_del_init(&req->node);
		ipc_dev->nr_pending--;
	}
	spin_unlock_irqrestore(&ipc_dev->list_lock, flags);
	if (req)
		starlet_ipc_start_request(req);
	else
		clear_bit(__TX_INUSE, &ipc_dev->flags);

	return IRQ_HANDLED;
}

/*
 * Receive Buffer Full Interrupt.
 */
static int starlet_ipc_dispatch_rbfi(struct starlet_ipc_device *ipc_dev)
{
	void __iomem *io_base = ipc_dev->io_base;
	struct starlet_ipc_request *req;
	unsigned long req_bus_addr;

	DBG("rx buf full interrupt!\n");

	req_bus_addr = starlet_ipc_recvfrom(io_base);
	req = starlet_ipc_find_request_by_bus_addr(ipc_dev, req_bus_addr);
	if (req) {
		starlet_ipc_complete_request(req);
	} else {
		ipc_printk(KERN_WARNING, "unknown request, bus=%p\n",
			   (void *)req_bus_addr);
	}
	return IRQ_HANDLED;
}

typedef int (*ipc_handler_t) (struct starlet_ipc_device *);

/*
 *
 */
static int
starlet_ipc_cond_dispatch_irq(struct starlet_ipc_device *ipc_dev,
			      u32 irqmask, u32 irq, ipc_handler_t handler)
{
	void __iomem *io_base = ipc_dev->io_base;
	u32 csr;
	int retval = IRQ_NONE;

	csr = in_be32(io_base + STARLET_IPC_CSR);
	if ((csr & (irqmask | irq)) == (irqmask | irq)) {
		retval = handler(ipc_dev);
		if (retval == IRQ_HANDLED) {
			starlet_ipc_update_csr(io_base, irq);
			starlet_ipc_eoi(io_base);
		}
	}
	return retval;
}

/*
 *
 */
static irqreturn_t starlet_ipc_handler(int irq, void *data)
{
	struct starlet_ipc_device *ipc_dev = (struct starlet_ipc_device *)data;
	int retval;

	/* starlet read a request */
	retval = starlet_ipc_cond_dispatch_irq(ipc_dev,
					       STARLET_IPC_CSR_TBEIMASK,
					       STARLET_IPC_CSR_TBEI,
					       starlet_ipc_dispatch_tbei);
	if (retval == IRQ_HANDLED)
		return retval;

	/* starlet wrote a reply */
	retval = starlet_ipc_cond_dispatch_irq(ipc_dev,
					       STARLET_IPC_CSR_RBFIMASK,
					       STARLET_IPC_CSR_RBFI,
					       starlet_ipc_dispatch_rbfi);
	return retval;
}

/*
 * IPC Calls.
 *
 */

/*
 * Internal. Completion routine.
 */
static void starlet_ipc_wait_done(struct starlet_ipc_request *req)
{
	complete(req->done_data);
}

/*
 *
 */
static int starlet_ipc_call_and_wait(struct starlet_ipc_request *req)
{
	DECLARE_COMPLETION(complete);

	req->done_data = &complete;
	req->done = starlet_ipc_wait_done;
	starlet_ipc_submit_request(req);
	wait_for_completion(&complete);
	return req->result;
}

/*
 *
 * IOS High level interfaces.
 */

static struct starlet_ipc_device *starlet_ipc_device_instance;

/*
 *
 */
struct starlet_ipc_device *starlet_ipc_get_device(void)
{
	if (!starlet_ipc_device_instance)
		ipc_printk(KERN_ERR, "uninitialized device instance!\n");
	return starlet_ipc_device_instance;
}
EXPORT_SYMBOL_GPL(starlet_ipc_get_device);


/* private aligned buffer for device pathnames */
DEFINE_MUTEX(buf_aligned_pathname_lock);
static char buf_aligned_pathname[64]
    __attribute__ ((aligned(STARLET_IPC_DMA_ALIGN + 1)));

/*
 *
 */
int starlet_ios_open(const char *pathname, int flags)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	struct starlet_ipc_request *req;
	dma_addr_t dma_addr;
	void *vaddr = (void *)pathname;
	size_t len;
	int use_private_buf;
	int retval = -ENOMEM;

	if (!ipc_dev)
		return -ENODEV;

	len = strlen(pathname) + 1;
	use_private_buf = !IS_ALIGNED((unsigned long)pathname,
				      STARLET_IPC_DMA_ALIGN + 1);

	req = starlet_ipc_alloc_request(ipc_dev);
	if (req) {
		if (use_private_buf) {
			mutex_lock(&buf_aligned_pathname_lock);
			/* FIXME: check pathname length */
			strcpy(buf_aligned_pathname, pathname);
			vaddr = buf_aligned_pathname;
		}
		dma_addr = dma_map_single(&ipc_dev->pdev.dev,
					  vaddr, len, DMA_TO_DEVICE);
		req->cmd = STARLET_IOS_OPEN;
		req->open.pathname = dma_addr;	/* bus address */
		req->open.mode = flags;
		retval = starlet_ipc_call_and_wait(req);
		if (use_private_buf) {
			dma_unmap_single(&ipc_dev->pdev.dev, dma_addr,
					 len, DMA_TO_DEVICE);
			mutex_unlock(&buf_aligned_pathname_lock);
		}
		starlet_ipc_free_request(req);
	}
	return retval;
}
EXPORT_SYMBOL_GPL(starlet_ios_open);

/*
 *
 */
int starlet_ios_close(int fd)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	struct starlet_ipc_request *req;
	int retval = -ENOMEM;

	if (!ipc_dev)
		return -ENODEV;

	req = starlet_ipc_alloc_request(ipc_dev);
	if (req) {
		req->cmd = STARLET_IOS_CLOSE;
		req->fd = fd;
		retval = starlet_ipc_call_and_wait(req);
		starlet_ipc_free_request(req);
	}
	return retval;
}
EXPORT_SYMBOL_GPL(starlet_ios_close);

/*
 *
 */
int starlet_ios_ioctl(int fd, int request,
		      dma_addr_t ibuf, size_t ilen,
		      dma_addr_t obuf, size_t olen)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	struct starlet_ipc_request *req;
	int retval = -EINVAL;

	if (!ipc_dev)
		return -ENODEV;

	req = starlet_ipc_alloc_request(ipc_dev);
	if (req) {
		req->cmd = STARLET_IOS_IOCTL;
		req->fd = fd;
		req->ioctl.request = (u32) request;
		req->ioctl.ibuf = ibuf;
		req->ioctl.ilen = ilen;
		req->ioctl.obuf = obuf;
		req->ioctl.olen = olen;
		retval = starlet_ipc_call_and_wait(req);
		starlet_ipc_free_request(req);
	}
	return retval;
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctl);

/*
 * Platform driver interface.
 *
 */

/*
 *
 */
static int starlet_ipc_init_irq(struct starlet_ipc_device *ipc_dev)
{
	int retval;
	int irq = platform_get_irq(&ipc_dev->pdev, 0);

	retval = request_irq(irq, starlet_ipc_handler, 0, DRV_MODULE_NAME,
			     ipc_dev);
	if (retval) {
		ipc_printk(KERN_ERR, "request of irq%d failed\n", irq);
	} else {
		/* ack and enable MBOX? and REPLY interrupts */
		out_be32(ipc_dev->io_base + STARLET_IPC_CSR,
			 STARLET_IPC_CSR_TBEIMASK | STARLET_IPC_CSR_RBFIMASK |
			 STARLET_IPC_CSR_TBEI | STARLET_IPC_CSR_RBFI);
	}
	return retval;
}

/*
 *
 */
static void starlet_ipc_exit_irq(struct starlet_ipc_device *ipc_dev)
{
	int irq = platform_get_irq(&ipc_dev->pdev, 0);

	starlet_ipc_quiesce(ipc_dev);
	free_irq(irq, ipc_dev);
}

/*
 *
 */
static int starlet_ipc_probe(struct platform_device *pdev)
{
	struct starlet_ipc_device *ipc_dev = to_ipc_dev(pdev);
	struct resource *mem;
	size_t size;
	int retval;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		ipc_printk(KERN_ERR, "failed to determine io address range\n");
		return -ENODEV;
	}

	memset(ipc_dev, 0, sizeof(*ipc_dev) - sizeof(ipc_dev->pdev));

	ipc_dev->io_base = ioremap(mem->start, mem->end - mem->start + 1);
	if (!ipc_dev->io_base) {
		ipc_printk(KERN_ERR, "ioremap from %p to %p failed\n",
			   (void *)mem->start, (void *)mem->end);
		return -EINVAL;
	}

	size = max((size_t) 128, sizeof(struct starlet_ipc_request));
	ipc_dev->dma_pool = dma_pool_create(DRV_MODULE_NAME,
					    &ipc_dev->pdev.dev,
					    size, STARLET_IPC_DMA_ALIGN + 1, 0);
	if (!ipc_dev->dma_pool) {
		ipc_printk(KERN_ERR, "dma_pool_create failed\n");
		iounmap(ipc_dev->io_base);
		return -ENOMEM;
	}
	spin_lock_init(&ipc_dev->list_lock);
	INIT_LIST_HEAD(&ipc_dev->pending_list);
	INIT_LIST_HEAD(&ipc_dev->outstanding_list);

	starlet_ipc_device_instance = ipc_dev;

	retval = starlet_ipc_init_irq(ipc_dev);
	if (retval) {
		starlet_ipc_device_instance = NULL;
		dma_pool_destroy(ipc_dev->dma_pool);
		iounmap(ipc_dev->io_base);
		return retval;
	}

	return 0;
}

/*
 *
 */
static int starlet_ipc_remove(struct platform_device *pdev)
{
	struct starlet_ipc_device *ipc_dev = to_ipc_dev(pdev);

	iounmap(ipc_dev->io_base);
	starlet_ipc_exit_irq(ipc_dev);

	return 0;
}

static struct platform_driver starlet_ipc_driver = {
	.probe = starlet_ipc_probe,
	.remove = starlet_ipc_remove,
	.driver = {
		   .name = DRV_MODULE_NAME,
		   },
};

static struct resource starlet_ipc_resources[] = {
	[0] = {
	       .start = STARLET_IPC_BASE,
	       .end = STARLET_IPC_BASE + STARLET_IPC_SIZE - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = STARLET_IPC_IRQ,
	       .end = STARLET_IPC_IRQ,
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct starlet_ipc_device starlet_ipc_device = {
	.pdev = {
		 .name = DRV_MODULE_NAME,
		 .id = 0,
		 .num_resources = ARRAY_SIZE(starlet_ipc_resources),
		 .resource = starlet_ipc_resources,
		 },
};

/*
 * Kernel module interface.
 *
 */

/*
 *
 */
static int __init starlet_ipc_init(void)
{
	int retval;

	ipc_printk(KERN_INFO, "%s - version %s\n", DRV_DESCRIPTION,
		   starlet_ipc_driver_version);

	retval = platform_driver_register(&starlet_ipc_driver);
	if (!retval) {
		retval = platform_device_register(&starlet_ipc_device.pdev);
		if (retval)
			platform_driver_unregister(&starlet_ipc_driver);
	}
	return retval;
}

/*
 *
 */
static void __exit starlet_ipc_exit(void)
{
	platform_device_unregister(&starlet_ipc_device.pdev);
	platform_driver_unregister(&starlet_ipc_driver);
}

module_init(starlet_ipc_init);
module_exit(starlet_ipc_exit);
