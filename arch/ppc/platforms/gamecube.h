/*
 * arch/ppc/platforms/gamecube.h
 *
 * Nintendo GameCube board-specific definitions.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MACH_GAMECUBE_H
#define __MACH_GAMECUBE_H

#include <asm/ppcboot.h>

/*
 * We have a total of 14 IRQs. Each has a corresponding bit in both
 * the Interrupt Cause (PIIC) and Interrupt Mask (PIIM) registers.
 */
#define GAMECUBE_IRQS		14
#define GAMECUBE_PIIC		((volatile ulong *)0xCC003000)
#define GAMECUBE_PIIM		((volatile ulong *)0xCC003004)

/*
 * Anything written here automagically puts us through reset.
 */
#define GAMECUBE_RESET		0xCC003024

/*
 * Until a driver for gcdvd exists, these may seek refuge here.
 */
#define GAMECUBE_DICVR          0xcc006004 /* DI Cover Register */
#define GC_DI_DISR              0xcc006000 /* DI Status Register */
#define GC_DI_DISR_BRKINT       (1<<6)
#define GC_DI_DISR_BRKINTMASK   (1<<5)
#define GC_DI_DISR_TCINT        (1<<4)
#define GC_DI_DISR_TCINTMASK    (1<<3)
#define GC_DI_DISR_DEINT        (1<<2)
#define GC_DI_DISR_DEINTMASK    (1<<1)
#define GC_DI_DISR_BRK          (1<<0)

#define GAMECUBE_IN(a) (*(volatile unsigned long *)a)
#define GAMECUBE_OUT(a,d) (*(volatile unsigned long *)a = d)

#endif /* !__MACH_GAMECUBE_H */
