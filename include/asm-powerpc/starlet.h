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


#define STARLET_IPC_DMA_ALIGN   0x1f /* 32 bytes */

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

	struct device *dev;

};

/* from starlet-ipc.c */

extern struct starlet_ipc_device *starlet_ipc_get_device(void);

extern int starlet_ios_open(const char *pathname, int flags);
extern int starlet_ios_close(int fd);
extern int starlet_ios_ioctl(int fd, int request,
			     void *ibuf, size_t ilen, 
			     void *obuf, size_t olen);

/* from starlet-stm.c */

extern void starlet_stm_restart(void);
extern void starlet_stm_power_off(void);

#endif /* __ASM_POWERPC_STARLET_H */
