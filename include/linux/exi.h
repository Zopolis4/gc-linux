/*
 * include/linux/exi.h
 *
 * Nintendo GameCube EXI driver 
 * Copyright (C) 2004-2005 The GameCube Linux Team
 * Copyright (C) 2004,2005 Todd Jeffreys <todd@voidpointer.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __exi_bus__
#define __exi_bus__

#define EXI_DMA_ALIGNMENT  32
/* -------------------
   exi_sub_command flags
   ----------------- */
#define EXI_CMD_READ   (0x00000000)
#define EXI_CMD_WRITE  (0x00000005)


/* ----------------------
   exi_command flags 
   --------------------- */

#define EXI_DESELECT_UDELAY (0x00000001)

#include <linux/device.h>
#include <linux/list.h>

struct exi_command
{
	unsigned int flags;
	
	void *data;
	unsigned int len;
	
	void *param;
	void (*completion_routine)(struct exi_command *cmd);
};

struct exi_command_group
{
	struct list_head list;
	
	struct exi_device *dev;
	
	unsigned int flags;
	
	unsigned int deselect_udelay;
	
	unsigned int num_commands;
	struct exi_command *commands;
};

extern struct bus_type exi_bus_type;

struct exi_device_id 
{
	unsigned int channel;
	unsigned int device;
	
	u32 id;
};

struct exi_device
{
	struct exi_device_id eid;

	struct device dev;
};

#define to_exi_device(n) container_of(n,struct exi_device,dev)

struct exi_driver
{
	char *name;
	struct exi_device_id eid;
	unsigned int frequency;
	
	int  (*probe)  (struct exi_device *dev);
	void (*remove) (struct exi_device *dev);

	struct device_driver driver;
};

typedef int (*exi_irq_handler)(int channel,void *param);

#define to_exi_driver(n) container_of(n,struct exi_driver,driver)

void exi_add_command_group(struct exi_command_group *cmd,unsigned int count);

int  exi_register_driver(struct exi_driver *drv);
void exi_unregister_driver(struct exi_driver *drv);

int  exi_register_irq(int channel_irq,exi_irq_handler func,void *param);
void exi_unregister_irq(int channel_irq);

#define exi_get_driver_data(exi_device) dev_get_drvdata(&exi_device->dev)
#define exi_set_driver_data(exi_device,data) dev_set_drvdata(&exi_device->dev,data)

#endif
