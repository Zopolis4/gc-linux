/*
 * include/linux/exi.h
 */

#ifndef __EXI_H
#define __EXI_H

#include <linux/device.h>


struct exi_dev {
	struct device dev;
};

#define to_exi_dev(n)		container_of(n, struct exi_dev, dev)

struct exi_driver {
	char *name;

	struct device_driver driver;
};

#define to_exi_driver(drv)	container_of(drv, struct exi_driver, driver)


extern struct bus_type exi_bus_type;

extern int exi_driver_register(struct exi_driver *drv);
extern void exi_driver_unregister(struct exi_driver *drv);

#endif	/* !__EXI_H */
