/*
 * drivers/exi/exi-bus.c
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/exi.h>
#include <asm/io.h>
#include "exi_priv.h"

static int exi_match(struct device *dev,struct device_driver *drv);


struct exi_interrupt_handlers irq_handlers[EXI_MAX_CHANNELS];
struct exi_locked_data exi_data[EXI_MAX_CHANNELS];

static struct exi_device exi_devices[EXI_MAX_CHANNELS][EXI_DEVICES_PER_CHANNEL];
static struct device exi_parent[EXI_MAX_CHANNELS] = {
	{ 
		.bus_id = "exi0", 
	},
	{ 
		.bus_id = "exi1", 
	},
	{ 
		.bus_id = "exi2",
	}
};

static struct bus_type exi_bus_type = {
	.match = exi_match,
	.name = "exi"
};

int exi_register_irq(int channel_irq,exi_irq_handler func,void *param)
{
	void* __iomem reg;
	u32 csr;
	if (channel_irq < EXI_MAX_CHANNELS) {
		
		if (irq_handlers[channel_irq].func) {
			return -EBUSY;
		}
		
		irq_handlers[channel_irq].param = param;
		irq_handlers[channel_irq].func = func;
		/* now setup the hardware */
		reg = EXI_CSR(channel_irq);
		csr = readl(reg);
		writel(csr | (EXI_CSR_EXIINT | EXI_CSR_EXIINTMASK),reg);
		return 0;
	}
	return -EINVAL;
}

void exi_unregister_irq(int channel_irq)
{
	void * __iomem reg;
	u32 csr;
	if (channel_irq < EXI_MAX_CHANNELS) {
		
		irq_handlers[channel_irq].func = NULL;
		
		reg = EXI_CSR(channel_irq);
		csr = readl(reg);
		writel((csr | EXI_CSR_EXIINT) & ~EXI_CSR_EXIINTMASK,reg);
	}
}

static int exi_device_probe(struct device *dev)
{
	struct exi_device *exi_device = to_exi_device(dev);
	struct exi_driver *exi_driver = to_exi_driver(dev->driver);
	int err = -ENODEV;
	
	if (exi_driver->probe) {
		err = exi_driver->probe(exi_device);
	}
	
	return err;
}

static int exi_device_remove(struct device *dev)
{
	struct exi_device *exi_device = to_exi_device(dev);
	struct exi_driver *exi_driver = to_exi_driver(dev->driver);

	if (exi_driver->remove) {
		exi_driver->remove(exi_device);
	}
	return 0;
}

int exi_register_driver(struct exi_driver *driver)
{
	driver->driver.name   = driver->name;
	driver->driver.bus    = &exi_bus_type;
	driver->driver.probe  = exi_device_probe;
	driver->driver.remove = exi_device_remove;

	return driver_register(&driver->driver);
}

void exi_unregister_driver(struct exi_driver *driver)
{
	driver_unregister(&driver->driver);
}

static int exi_match(struct device *dev,struct device_driver *drv) {
	/* return 1 if the driver can handle the device */
	struct exi_device *exi_device = to_exi_device(dev);
	struct exi_driver *exi_driver = to_exi_driver(drv);

	if ((exi_device->eid.channel == exi_driver->eid.channel) &&
	    (exi_device->eid.device  == exi_driver->eid.device) &&
	    (exi_device->eid.id      == exi_driver->eid.id)) {
		return 1;
	}
	return 0;
}

static void exi_bus_scan(void)
{
	unsigned int channel;
	unsigned int device;
	
	for (channel=0;channel<EXI_MAX_CHANNELS;++channel) {
		/* initialize the data per each exi bus */
		exi_data[channel].exi_state = EXI_IDLE;
		exi_data[channel].cur_command = 0;
		atomic_set(&exi_data[channel].tc_interrupt,0);
		spin_lock_init(&exi_data[channel].queue_lock);
		INIT_LIST_HEAD(&exi_data[channel].queue);
		tasklet_init(&exi_data[channel].tasklet,
			     exi_tasklet,
			     (unsigned long)&exi_data[channel]);

		/* add the exi devices underneath the parents */
		for (device=0;device<EXI_DEVICES_PER_CHANNEL;++device) {
			exi_devices[channel][device].eid.channel = channel;
			exi_devices[channel][device].eid.device  = device;
			
			sprintf(exi_devices[channel][device].dev.bus_id,
				"dev%u",device);
			exi_devices[channel][device].dev.parent = 
				&exi_parent[channel];
			exi_devices[channel][device].dev.bus = &exi_bus_type;
			exi_devices[channel][device].dev.platform_data = 
				&exi_data[channel];
			/* now ID the device */
			exi_devices[channel][device].eid.id = 
				exi_synchronous_id(channel,device);
			if (exi_devices[channel][device].eid.id != EXI_INVALID_ID) {
				printk(KERN_INFO "%s:%s: %x\n",
				       exi_parent[channel].bus_id,
				       exi_devices[channel][device].dev.bus_id,
				       exi_devices[channel][device].eid.id);
				
				device_register(&exi_devices[channel][device].dev);
			}
		}
	}
}

void exi_bus_insert(unsigned int channel,unsigned int bInsert)
{
	/* 
	   this is all wrong, just skip this function, no hot-plug support
	   for now 
	*/
	/*u32 device;
	u32 id;

	if (bInsert)
	{
		for (device=0;device<EXI_DEVICES_PER_CHANNEL;++device) {
			id = exi_synchronous_id(channel,device);
			if (id != EXI_INVALID_ID) {
				device_register(&exi_devices[channel][device].dev);
			}
		}
	}
	else {
		device_register(&exi_devices[channel][0].dev);
		device_register(&exi_devices[channel][1].dev);
		device_register(&exi_devices[channel][2].dev);
		}*/
}

static int __init exi_init(void)
{
	int i;
	int r;
	/* acquire the interrupt */
	printk(KERN_INFO "Initializing EXI interface\n");

	/* register the bus */
	if ((r=bus_register(&exi_bus_type))) {
		return r;
	}
	/* register root devices */
	for (i=0;i<EXI_MAX_CHANNELS;++i) {
		if ((r=device_register(exi_parent+i))) {
			return r;
		}
	}

	irq_handlers[0].func = NULL;
	irq_handlers[1].func = NULL;
	irq_handlers[2].func = NULL;
	
	for (r=0;r<EXI_MAX_CHANNELS;++r) {
		/* wait for all transfers to complete */
		while (readl(EXI_CR(r)) & EXI_MR_TSTART) { }
		/* clear interrupts and reset interrupts */
		writel(EXI_CSR_TCINT  | EXI_CSR_EXIINT | EXI_CSR_EXTINT |
		       EXI_CSR_EXTINTMASK,
		       EXI_CSR(r));
	}
	/* now enumerate through the bus and add all detected devices */
	exi_bus_scan();
	
	return (request_irq(EXI_IRQ,exi_bus_irq_handler,0,"EXI",NULL));
}

EXPORT_SYMBOL(exi_register_driver);
EXPORT_SYMBOL(exi_unregister_driver);
EXPORT_SYMBOL(exi_register_irq);
EXPORT_SYMBOL(exi_unregister_irq);
EXPORT_SYMBOL(exi_add_command_group);
EXPORT_SYMBOL(exi_bus_type);

postcore_initcall(exi_init);
