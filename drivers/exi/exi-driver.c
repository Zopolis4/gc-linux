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

/**
 *	exi_bus_match - attach a driver to a device.
 *	@dev: device structure to match.
 *	@drv: driver structure to match against.
 *
 *	Attaches a driver to a device by matching the device IDs
 *	the driver claims to support with the actual device ID of
 *	a particular device.
 *
 *	Returns 1 when driver is attached, 0 otherwise.
 */
static int exi_bus_match(struct device *dev, struct device_driver *drv)
{
	struct exi_dev *exi_dev = to_exi_dev(dev);
	struct exi_driver *exi_drv = to_exi_driver(drv);
	const struct exi_device_id *ids = exi_drv->id_table;

	if (!ids)
		return 0;

	while (ids->dev_id) {
		if (ids->dev_id == exi_dev->id)
			return 1;
		ids++;
	}

	return 0;
}


struct bus_type exi_bus_type = {
	.name		= "exi",
	.match		= exi_bus_match,
};

static int __init exi_driver_init(void)
{
	return bus_register(&exi_bus_type);
}

postcore_initcall(exi_driver_init);

EXPORT_SYMBOL(exi_bus_type);
EXPORT_SYMBOL(exi_driver_register);
EXPORT_SYMBOL(exi_driver_unregister);
