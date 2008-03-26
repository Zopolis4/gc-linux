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
#include <linux/of_platform.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <asm/io.h>
#include <asm/bitops.h>

#include <asm/starlet.h>


#define DRV_MODULE_NAME		"starlet-ipc"
#define DRV_DESCRIPTION		"Nintendo Wii starlet IPC driver"
#define DRV_AUTHOR		"Albert Herranz"

static char starlet_ipc_driver_version[] = "0.1-isobel";

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


/* starlet_ipc_device flags */
enum {
	__TX_INUSE = 0,		/* tx buffer in use flag */
};


struct starlet_ipc_request {
	/* begin starlet firmware request format */
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
	/* end starlet firmware request format */

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
	pr_debug("cmd=%x, result=%d, fd=%x, dma_addr=%p\n",
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

	pr_debug("start_request\n");
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

	pr_debug("complete_request\n");
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

	pr_debug("tx buf empty interrupt!\n");

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

	pr_debug("rx buf full interrupt!\n");

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
		drv_printk(KERN_ERR, "uninitialized device instance!\n");
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
		dma_addr = dma_map_single(ipc_dev->dev,
					  vaddr, len, DMA_TO_DEVICE);
		req->cmd = STARLET_IOS_OPEN;
		req->open.pathname = dma_addr;	/* bus address */
		req->open.mode = flags;
		retval = starlet_ipc_call_and_wait(req);
		if (use_private_buf) {
			dma_unmap_single(ipc_dev->dev, dma_addr,
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
int starlet_ios_ioctl_dma(int fd, int request,
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

/*
 *
 */
int starlet_ios_ioctl(int fd, int request,
		      void *ibuf, size_t ilen,
		      void *obuf, size_t olen)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	dma_addr_t ibuf_ba, obuf_ba;
	int retval;

	BUG_ON(!IS_ALIGNED((unsigned long)ibuf, STARLET_IPC_DMA_ALIGN+1));
	BUG_ON(!IS_ALIGNED((unsigned long)obuf, STARLET_IPC_DMA_ALIGN+1));

	ibuf_ba = dma_map_single(ipc_dev->dev, ibuf, ilen,
				 DMA_TO_DEVICE);
	obuf_ba = dma_map_single(ipc_dev->dev, obuf, olen,
				 DMA_FROM_DEVICE);
	retval = starlet_ios_ioctl_dma(fd, request, ibuf_ba, ilen,
				       obuf_ba, olen);
	dma_unmap_single(ipc_dev->dev, ibuf_ba, ilen, DMA_TO_DEVICE);
	dma_unmap_single(ipc_dev->dev, obuf_ba, olen, DMA_FROM_DEVICE);

	return retval;
}
EXPORT_SYMBOL_GPL(starlet_ios_ioctl);

/*
 *
 *
 */

/*
 *
 */
static void starlet_ios_fixups(void)
{
	int fd;
	static u32 buf[32/sizeof(u32)]
	    __attribute__ ((aligned(STARLET_IPC_DMA_ALIGN + 1)));

	/* try to close any open file descriptors, just in case */
	for(fd = 0; fd < 15; fd++)
		starlet_ios_close(fd);

	/* stop dvd motor */
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
}


/*
 *
 */
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

	/* ack and enable MBOX? and REPLY interrupts */
	out_be32(ipc_dev->io_base + STARLET_IPC_CSR,
		 STARLET_IPC_CSR_TBEIMASK | STARLET_IPC_CSR_RBFIMASK |
		 STARLET_IPC_CSR_TBEI | STARLET_IPC_CSR_RBFI);

	starlet_ios_fixups();

	return retval;
}

/*
 *
 */
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
 * Device interface.
 *
 */

/*
 * Common probe function for the given device.
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

/*
 *
 */
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

/*
 *
 */
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
 * OF Platform device interface.
 *
 */

/*
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

/*
 *
 */
static int starlet_ipc_of_remove(struct of_device *odev)
{
	return starlet_ipc_do_remove(&odev->dev);
}

/*
 *
 */
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
 * Kernel module interface.
 *
 */

/*
 *
 */
static int __init starlet_ipc_init_module(void)
{
	drv_printk(KERN_INFO, "%s - version %s\n", DRV_DESCRIPTION,
		   starlet_ipc_driver_version);

	return of_register_platform_driver(&starlet_ipc_of_driver);
}

/*
 *
 */
static void __exit starlet_ipc_exit_module(void)
{
	of_unregister_platform_driver(&starlet_ipc_of_driver);
}

module_init(starlet_ipc_init_module);
module_exit(starlet_ipc_exit_module);


