/*
 * arch/powerpc/platforms/embedded6xx/starlet-stm.c
 *
 * Nintendo Wii starlet STM routines
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
#include <linux/dma-mapping.h>
#include <asm/starlet.h>

/*
 * /dev/stm/immediate
 *
 */

#define STARLET_STM_HOTRESET	0x2001
#define STARLET_STM_SHUTDOWN	0x2003

#define STARLET_DEV_STM_IMMEDIATE	"/dev/stm/immediate"


static const char dev_stm_immediate[] = STARLET_DEV_STM_IMMEDIATE;

/* private aligned buffer for restart/power_off operations */
static u32 starlet_stm_buf[(STARLET_IPC_DMA_ALIGN+1)/sizeof(u32)]
		 __attribute__ ((aligned (STARLET_IPC_DMA_ALIGN+1)));

/*
 *
 */
static void starlet_stm_common_restart(int request, u32 value)
{
	struct starlet_ipc_device *ipc_dev = starlet_ipc_get_device();
	dma_addr_t dma_addr;
	u32 *vaddr = starlet_stm_buf;
	size_t len = sizeof(starlet_stm_buf);
	int fd;

	if (!ipc_dev)
		return;

	fd = starlet_ios_open(dev_stm_immediate, 0);
	if (fd >= 0) {
		*vaddr = value;
		dma_addr = dma_map_single(&ipc_dev->pdev.dev,
					  vaddr, len, DMA_BIDIRECTIONAL);
		starlet_ios_ioctl(fd, request, dma_addr, len, dma_addr, len);
		dma_unmap_single(&ipc_dev->pdev.dev,
				 dma_addr, len, DMA_BIDIRECTIONAL);
		starlet_ios_close(fd);
	}
}

/*
 *
 */
void starlet_stm_restart(void)
{
	starlet_stm_common_restart(STARLET_STM_HOTRESET, 0);
}
//EXPORT_SYMBOL_GPL(starlet_stm_restart);

/*
 *
 */
void starlet_stm_power_off(void)
{
	starlet_stm_common_restart(STARLET_STM_SHUTDOWN, 0);
}
//EXPORT_SYMBOL_GPL(starlet_stm_power_off);


