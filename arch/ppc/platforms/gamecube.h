/*
 * arch/ppc/platforms/gamecube.h
 *
 * Nintendo GameCube board-specific definitions
 * Copyright (C) 2004 The GameCube Linux Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __MACH_GAMECUBE_H
#define __MACH_GAMECUBE_H

#include <asm/ppcboot.h>

/*
 * This is the current memory layout for the GameCube Linux port.
 *
 *   +------------------------------+ 
 *   | framebuffer  640x576x2 bytes | GCN_XFB_END
 *   .                              .
 *   .                              .
 *   +------------------------------+ GCN_XFB_START
 *   | kexec reserved  4x4096 bytes | GCN_KXC_END
 *   .                              .
 *   +------------------------------+ GCN_KXC_START
 *   | memory       remaining bytes | GCN_MEM_END
 *   .                              .
 *   .                              .
 *   .                              .
 *   +- - - - - - - - - - - - - - - + 
 *   | Dolphin OS       12544 bytes |
 *   | globals, pre-kernel          |
 *   |                              |
 *   |                              |
 *   +------------------------------+ GCN_MEM_START
 *
 */

/*
 * Some useful sizes
 */
#define GCN_RAM_SIZE            (24*1024*1024) /* 24 MB */
#define GCN_XFB_SIZE            (640*576*2)    /* pal framebuffer */
#ifdef CONFIG_KEXEC
  #define GCN_KXC_SIZE          (4*4096) /* PAGE_ALIGN(GCN_PRESERVE_SIZE) */
#else
  #define GCN_KXC_SIZE          (0)
#endif
#define GCN_MEM_SIZE            (GCN_MEM_END+1)

/*
 * Start and end of several regions
 */
#define GCN_XFB_END             (GCN_RAM_SIZE-1)
#define GCN_XFB_START           (GCN_XFB_END-GCN_XFB_SIZE+1)
#define GCN_KXC_END             (GCN_XFB_START-1)
#define GCN_KXC_START           (GCN_KXC_END-GCN_KXC_SIZE+1)
#define GCN_MEM_END             (GCN_KXC_START-1)
#define GCN_MEM_START           (0x00000000)

/*
 * Some memory regions will be preserved across kexec reboots, if enabled.
 */
#define GCN_PRESERVE_START      (0x00000000)
#define GCN_PRESERVE_END        (0x000030ff)
#define GCN_PRESERVE_FROM       (GCN_PRESERVE_START)
#define GCN_PRESERVE_TO         (GCN_KXC_START)
#define GCN_PRESERVE_SIZE       (GCN_PRESERVE_END+1)

#endif /* !__MACH_GAMECUBE_H */

