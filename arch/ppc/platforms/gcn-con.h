/*
 * arch/ppc/platforms/gcn-con.h
 *
 * Nintendo GameCube early debug console definitions
 * Copyright (C) 2004 The GameCube Linux Team
 *
 * Based on console.h by tmbinc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __GCN_CON_H
#define __GCN_CON_H

void gcn_con_init(void);
void gcn_con_puts(const char *);
void gcn_con_putc(char);

#endif /* __GCN_CON_H */

