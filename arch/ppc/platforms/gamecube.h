/*
 * arch/ppc/platforms/gamecube.h
 *
 * We steal here
 * Macros, definitions, and data structures specific to the IBM PowerPC
 * STB03xxx "Redwood" evaluation board.
 *
 * Author: Armin Kuster <akuster@mvista.com>
 *
 * 2001 (c) MontaVista, Software, Inc.  This file is licensed under
 * the terms of the GNU General Public License version 2.  This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifdef __KERNEL__
#ifndef __ASM_GAMECUBE_H__
#define __ASM_GAMECUBE_H__

#ifndef __ASSEMBLY__
typedef struct board_info {
	unsigned char	bi_s_version[4];	/* Version of this structure */
	unsigned char	bi_r_version[30];	/* Version of the IBM ROM */
	unsigned int	bi_memsize;		/* DRAM installed, in bytes */
	unsigned int	bi_dummy;		/* field shouldn't exist */
	unsigned char	bi_enetaddr[6];		/* Ethernet MAC address */
	unsigned int	bi_intfreq;		/* Processor speed, in Hz */
	unsigned int	bi_busfreq;		/* Bus speed, in Hz */
	unsigned int	bi_tbfreq;		/* Software timebase freq */
	unsigned int	bi_opb_busfreq;		/* OPB Bus speed, in Hz */
	int		bi_iic_fast[2];		/* Use fast i2c mode */
} bd_t;
#endif /* !__ASSEMBLY__ */

#define BASE_BAUD		(378000000 / 18 / 16)

#define GAMECUBE_PIIC 0xcc003000 /* PI interrupt cause */
#define GAMECUBE_PIIM 0xcc003004 /* PI interrupt mask */
#define GAMECUBE_RESET 0xcc003024 /* RESET */
#define GAMECUBE_DICVR 0xcc006004 /* DI Cover Register */

#define GAMECUBE_IN(a) (*(u_int *)a)
#define GAMECUBE_OUT(a,d) (*(u_int *)a = d)

#define GAMECUBE_IRQS 14

#endif /* __ASM_GAMECUBE_H__ */
#endif /* __KERNEL__ */
