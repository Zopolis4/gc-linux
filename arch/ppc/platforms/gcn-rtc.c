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

#ifndef EXI_LITE
#error Sorry, this driver needs currently the gcn-exi-lite framework.
#endif

/*
 * The EXI functions that we use are guaranteed to work by the gcn-exi-lite
 * framework even if exi_lite_init() has not been called.
 */

#define RTC_OFFSET 946684800L

static int bias = 0;

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

/**
 *
 */
long __init gcn_time_init(void)
{
	char sram[64];
	int *pbias = (int *)&sram[0xC];
	read_sram(sram);
	bias = *pbias;
	return 0;
}

/**
 *
 */
unsigned long gcn_get_rtc_time(void)
{
	return get_rtc() + bias + RTC_OFFSET;
}

/**
 *
 */
int gcn_set_rtc_time(unsigned long nowtime)
{
	set_rtc(nowtime - RTC_OFFSET - bias);

	return 1;
}

