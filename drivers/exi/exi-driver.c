/*
 * drivers/exi/exi-driver.c
 *
 * Nintendo GameCube Expansion Interface support. Driver model routines.
 * Copyright (C) 2004 Arthur Othieno <a.othieno@bluewin.ch>
 * Copyright (C) 2004 The GameCube Linux Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#define DEBUG

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/exi.h>


static int exi_device_probe(struct device *dev)
{
	struct exi_dev *exi_dev = to_exi_dev(dev);
	struct exi_driver *drv = to_exi_driver(dev->driver);
	int err = -ENODEV;

	if (drv->probe)
		err = drv->probe(exi_dev);

	return err;
}

static int exi_device_remove(struct device *dev)
{
	struct exi_dev *exi_dev = to_exi_dev(dev);
	struct exi_driver *drv = to_exi_driver(dev->driver);

	if (drv->remove)
		drv->remove(exi_dev);

	return 0;
}

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
	drv->driver.probe = exi_device_probe;
	drv->driver.remove = exi_device_remove;

	return driver_register(&drv->driver);
}

EXPORT_SYMBOL(exi_driver_register);

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

EXPORT_SYMBOL(exi_driver_unregister);


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

EXPORT_SYMBOL(exi_bus_type);

struct device exi_bus_dev = {
	.bus_id		= "exi0",
};

EXPORT_SYMBOL(exi_bus_dev);

static int __init exi_driver_init(void)
{
	int err;

	if ((err = device_register(&exi_bus_dev)))
		goto out;
	if ((err = bus_register(&exi_bus_type)))
		goto out;
out:
	return err;
}

postcore_initcall(exi_driver_init);
