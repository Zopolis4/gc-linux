#ifndef __EXI_H
#define __EXI_H

/*
 * include/linux/exi.h
 */

#include <linux/device.h>


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

#endif	/* !__EXI_H */
