/*
 * arch/ppc/platforms/gamecube.h
 *
 * We steal here
 * Macros, definitions, and data structures specific to the IBM PowerPC
 * STB03xxx "Redwood" evaluation board.
 *
 * Author: Armin Kuster <akuster@mvista.com>
 *
 * Albert Herranz
 *	GAMECUBE
 *	- Added some defines for DI interrupt handling.
 *
 * 2001 (c) MontaVista, Software, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __MACH_GAMECUBE_H
#define __MACH_GAMECUBE_H

#include <asm/ppcboot.h>


#define GAMECUBE_PIIC 0xcc003000 /* PI interrupt cause */
#define GAMECUBE_PIIM 0xcc003004 /* PI interrupt mask */
#define GAMECUBE_RESET 0xcc003024 /* RESET */
#define GAMECUBE_DICVR 0xcc006004 /* DI Cover Register */

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

#define GAMECUBE_IRQS 14

#endif /* !__MACH_GAMECUBE_H */
