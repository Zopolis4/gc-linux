/*
 * include/asm-powerpc/starlet.h
 *
 * Nintendo Wii starlet processor definitions
 * Copyright (C) 2008 The GameCube Linux Team
 * Copyright (C) 2008 Albert Herranz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __ASM_POWERPC_STARLET_H
#define __ASM_POWERPC_STARLET_H

#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/dmapool.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>


#define STARLET_IPC_DMA_ALIGN   0x1f /* 32 bytes */

struct starlet_ipc_request;

struct starlet_ipc_device {
	unsigned long flags;

	void __iomem *io_base;
	int irq;

	struct dma_pool *dma_pool;

	spinlock_t list_lock;
	struct list_head outstanding_list;
	unsigned long nr_outstanding;
	struct list_head pending_list;
	unsigned long nr_pending;

	struct starlet_ipc_request *req; /* for requests causing a ios reboot */

	struct device *dev;
};

struct starlet_iovec {
	dma_addr_t dma_addr;
	u32 dma_len;
};

typedef int (*starlet_ipc_callback_t)(struct starlet_ipc_request *req);

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
		struct {
			u32 request;
			u32 argc_in;
			u32 argc_out;
			dma_addr_t iovec_da;
		} ioctlv;
		u32 argv[5];
	};
	/* end starlet firmware request format */

	dma_addr_t dma_addr;	/* request dma address */

	/* ioctlv related data */
	struct starlet_iovec *iovec;
	size_t iovec_size;

	struct scatterlist *sgl_in;
	unsigned sgl_nents_in;
	struct scatterlist *sgl_out;
	unsigned sgl_nents_out;

	void *done_data;
	starlet_ipc_callback_t done;

	struct list_head node; /* for queueing */

	struct starlet_ipc_device *ipc_dev;
};



/* from starlet-ipc.c */

extern void *starlet_kzalloc(size_t size, gfp_t flags);
extern void starlet_kfree(void *ptr);

extern struct starlet_ipc_device *starlet_ipc_get_device(void);
extern void *starlet_ipc_request_priv(struct starlet_ipc_request *req);
extern void starlet_ipc_free_request(struct starlet_ipc_request *req);


extern int starlet_ios_open(const char *pathname, int flags);
extern int starlet_ios_close(int fd);

extern int starlet_ios_ioctl(int fd, int request,
			     void *ibuf, size_t ilen, 
			     void *obuf, size_t olen);
extern int starlet_ios_ioctl_nowait(int fd, int request,
				    void *ibuf, size_t ilen, 
				    void *obuf, size_t olen,
				    starlet_ipc_callback_t callback,
				    void *arg);
extern void starlet_ios_ioctl_complete(struct starlet_ipc_request *req);

extern int starlet_ios_ioctlv(int fd, int request,
			      unsigned int nents_in,
			      struct scatterlist *sgl_in,
			      unsigned int nents_out,
			      struct scatterlist *sgl_out);
extern int starlet_ios_ioctlv_nowait(int fd, int request,
				     unsigned int nents_in,
				     struct scatterlist *sgl_in,
				     unsigned int nents_out,
				     struct scatterlist *sgl_out,
				     starlet_ipc_callback_t callback,
				     void *arg);
extern int starlet_ios_ioctlv_and_reboot(int fd, int request,
					 unsigned int nents_in,
					 struct scatterlist *sgl_in,
					 unsigned int nents_out,
					 struct scatterlist *sgl_out);
extern void starlet_ios_ioctlv_complete(struct starlet_ipc_request *req);

/* from starlet-stm.c */

extern void starlet_stm_restart(void);
extern void starlet_stm_power_off(void);

#endif /* __ASM_POWERPC_STARLET_H */
