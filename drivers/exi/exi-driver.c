/*
 * drivers/exi/exi-driver.c
 *
 * Copyright (C) 2004  Arthur Othieno <a.othieno@bluewin.ch>
 */

#define DEBUG

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/exi.h>


/**
 *	exi_driver_register - register an EXI device driver.
 *	@drv: driver structure to register.
 *
 *	Registers an EXI device driver with the bus
 *	and consequently with the driver model core.
 */
int exi_driver_register(struct exi_driver *drv)
{
	drv->driver.name = drv->name;
	drv->driver.bus = &exi_bus_type;

	return driver_register(&drv->driver);
}

/**
 *	exi_driver_unregister - unregister an EXI device driver.
 *	@drv: driver structure to unregister.
 *
 *	Unregisters an EXI device driver with the bus
 *	and consequently with the driver model core.
 */
void exi_driver_unregister(struct exi_driver *drv)
{
	driver_unregister(&drv->driver);
}


struct bus_type exi_bus_type = {
	.name		= "exi",
};

static int __init exi_driver_init(void)
{
	return bus_register(&exi_bus_type);
}

postcore_initcall(exi_driver_init);

EXPORT_SYMBOL(exi_bus_type);
EXPORT_SYMBOL(exi_driver_register);
EXPORT_SYMBOL(exi_driver_unregister);
