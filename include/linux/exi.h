/*
 * include/linux/exi.h
 *
 * Nintendo GameCube EXpansion Interface definitions
 * Copyright (C) 2004 Arthur Othieno <a.othieno@bluewin.ch>
 * Copyright (C) 2004 The GameCube Linux Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */
#ifndef __EXI_H
#define __EXI_H

#include <linux/device.h>

/* while the real framework gets finished, we'll use the lite version */
#include <../drivers/exi/gcn-exi-lite.h>


struct exi_dev {
	unsigned long		id;

	struct device		dev;
};

#define to_exi_dev(n)		container_of(n, struct exi_dev, dev)


struct exi_device_id {
	unsigned long		dev_id;
};

struct exi_driver {
	char			*name;
	struct exi_device_id	*id_table;

	int	(*probe)	(struct exi_dev *dev);
	void	(*remove)	(struct exi_dev *dev);

	struct device_driver	driver;
};

#define to_exi_driver(drv)	container_of(drv, struct exi_driver, driver)


extern struct device exi_bus_dev;
extern struct bus_type exi_bus_type;

extern int exi_driver_register(struct exi_driver *drv);
extern void exi_driver_unregister(struct exi_driver *drv);

static inline void *exi_get_drvdata(struct exi_dev *exi_dev)
{
	return dev_get_drvdata(&exi_dev->dev);
}

static inline void exi_set_drvdata(struct exi_dev *exi_dev, void *data)
{
	dev_set_drvdata(&exi_dev->dev, data);
}

#endif	/* !__EXI_H */
