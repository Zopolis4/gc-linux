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
 * There are 14 IRQs in total. Each has a corresponding bit in both
 * the Interrupt Cause (ICR) and Interrupt Mask (IMR) registers.
 *
 * Enabling/disabling an interrupt line involves asserting/clearing
 * the corresponding bit in IMR. ACK'ing a request simply involves
 * asserting the corresponding bit in ICR.
 */
#define FLIPPER_ICR		((volatile ulong *)0xcc003000)
#define FLIPPER_IMR		((volatile ulong *)0xcc003004)

/*
 * Anything written here automagically puts us through reset.
 */
#define GAMECUBE_RESET		0xCC003024

#endif /* !__MACH_GAMECUBE_H */
