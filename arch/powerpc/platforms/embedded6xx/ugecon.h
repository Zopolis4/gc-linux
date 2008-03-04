/*
 * arch/powerpc/platforms/embedded6xx/ugecon.h
 *
 * USB Gecko early console on memcard slot B.
 * Copyright (C) 2008 The GameCube Linux Team
 * Copyright (C) 2008 Albert Herranz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __UGECON_H
#define __UGECON_H

#ifdef CONFIG_USBGECKO_EARLY_CONSOLE

extern void ug_early_putc(char ch);
extern void ug_early_puts(char *s);
extern void ug_early_con_init(void);

#endif /* CONFIG_USBGECKO_EARLY_CONSOLE */

#endif /* __UGECON_H */

