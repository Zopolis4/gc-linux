/*
 * arch/ppc/platforms/gcn-rtc.c
 *
 * Nintendo GameCube RTC functions
 * Copyright (C) 2004 The GameCube Linux Team
 *
 * Based on gamecube_time.c from Torben Nielsen.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/time.h>
#include <linux/exi.h>

/*
 * The EXI functions that we use are guaranteed to work by the gcn-exi-lite
 * framework even if exi_lite_init() has not been called.
 */

#define RTC_OFFSET 946684800L

static int rtc_probe(struct exi_device *);
static void rtc_remove(struct exi_device *);

static int bias = 0;
static int initialized = 0;
static struct exi_driver rtc_exi_driver = {
	.name = "RTC/SRAM",
	.eid = {
		.channel = 0,
		.device  = 1,
		.id      = 0xFFFF1698
	},
	.frequency = 3,
	.probe  = rtc_probe,
	.remove = rtc_remove 
};

#if 0
static void read_sram(unsigned char *abuf)
{
	unsigned long a;
	
	/* select the SRAM device */
	exi_select(0, 1, 3);

	/* send the appropriate command */
	a = 0x20000100;
	exi_write(0, &a, 4);

	/* read the SRAM data */
	exi_read(0, abuf, 64);

	/* deselect the SRAM device */
	exi_deselect(0);

	return;
}

static unsigned long get_rtc(void)
{
	unsigned long a = 0L;

	/* select the RTC device */
	exi_select(0, 1, 3);

	/* send the appropriate command */
	a = 0x20000000;
	exi_write(0, &a, 4);

	/* read the time and date value */
	exi_read(0, &a, 4);

	/* deselect the RTC device */
	exi_deselect(0);

	return a;
}

static void set_rtc(unsigned long aval)
{
	unsigned long a;

	/* select the RTC device */
	exi_select(0, 1, 3);

	/* send the appropriate command */
	a = 0xA0000000;
	exi_write(0, &a, 4);

	/* Set the new time and date value */
	exi_write(0, &aval, 4);

	/* Deselect the RTC device */
	exi_deselect(0);
}
#endif
/**
 *
 */
unsigned long gcn_get_rtc_time(void)
{
	static int i=0;
	if (i++ == 0) {
		printk(KERN_INFO "Get RTC time\n");
	}
	if (initialized) {
		//return get_rtc() + bias + RTC_OFFSET;
	}
	return 0;
}

/**
 *
 */
int gcn_set_rtc_time(unsigned long nowtime)
{
	printk(KERN_INFO "Set RTC time %lu\n",nowtime);
	if (initialized) {
		//set_rtc(nowtime - RTC_OFFSET - bias);
		return 0;
	}
	
	return 0;
}

static int rtc_probe(struct exi_device *dev) 
{
	/* nothing to probe, hardware is always there */
	initialized = 1;
	
	return 0;
}

static void rtc_remove(struct exi_device *dev)
{
	initialized = 0;
}

/**
 *
 */
long gcn_time_init(void)
{
	printk(KERN_INFO "gcn_time_init\n");
	initialized = 0;
	return 0;
	//return exi_register_driver(&rtc_exi_driver);
}
	/*
char sram[64];
	int *pbias = (int *)&sram[0xC];
	read_sram(sram);
	bias = *pbias;
	return 0; */

