/*
 * drivers/exi/gcn_exi_lite.h
 *
 * Nintendo GameCube EXpansion Interface support, "lite" version.
 * Copyright (C) 2004 The GameCube Linux Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __GCN_EXI_LITE_H
#define __GCN_EXI_LITE_H

#define MISSING_APGO_EXI_FRAMEWORK 1
#define EXI_LITE 1

int exi_lite_init(void);
void exi_lite_exit(void);

void exi_select(int channel, int device, int freq);
void exi_deselect(int channel);

void exi_read(int channel, void *data, int len);
void exi_write(int channel, void *data, int len);

#define EXI_EVENT_IRQ     0
#define EXI_EVENT_INSERT  1
#define EXI_EVENT_TC      2

int exi_register_event(int channel, int event_id,
		       int (*handler) (int, int, void *), void *dev);
int exi_unregister_event(int channel, int event_id);

#endif /* __GCN_EXI_LITE_H */
