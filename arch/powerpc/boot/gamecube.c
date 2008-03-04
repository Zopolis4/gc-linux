/*
 * arch/powerpc/boot/gamecube.c
 *
 * Nintendo GameCube boot 
 * Copyright (C) 2004-2008 The GameCube Linux Team
 * Copyright (C) 2008 Albert Herranz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#include <stddef.h>
#include "stdio.h"
#include "types.h"
#include "io.h"
#include "ops.h"

/*
 * Note that CONFIG_* defines are not available at this stage, except:
 * - CONFIG_USBGECKO_EARLY_CONSOLE
 * - CONFIG_GAMECUBE_WII
 */

BSS_STACK(4096);

/*
 * We enter with the MMU enabled and some legacy memory mappings active.
 *
 * We leave the MMU enabled, but we switch to an identity mapped memory
 * scheme as expected by the start code.
 *
 */
asm ("\n\
.text\n\
.globl _zimage_start\n\
_zimage_start:\n\
\n\
	isync\n\
	/* IBAT3,DBAT3 for first 16Mbytes */\n\
	li	8, 0x01ff	/* 16MB */\n\
	li      9, 0x0002	/* rw */\n\
	mtspr   0x216, 8	/* IBAT3U */\n\
	mtspr   0x217, 9	/* IBAT3L */\n\
	mtspr   0x21e, 8	/* DBAT3U */\n\
	mtspr   0x21f, 9	/* DBAT3L */\n\
\n\
	sync\n\
	isync\n\
\n\
	li	3, 0\n\
	li	4, 0\n\
	li	5, 0\n\
\n\
	bcl-    20,4*cr7+so,1f\n\
1:\n\
	mflr    8\n\
	clrlwi  8, 8, 3\n\
	addi    8, 8, 2f - 1b\n\
	mtlr    8\n\
	blr\n\
2:\n\
	b _zimage_start_lib\n\
");

#ifdef CONFIG_USBGECKO_EARLY_CONSOLE

#ifdef CONFIG_GAMECUBE_WII
#define EXI_BASE		(0xcd006800)
#else
#define EXI_BASE		(0xcc006800)
#endif

#define EXI_CHANNEL_SPACING	0x14

#define EXI_IO_BASE(c)	((void *)(EXI_BASE + ((c)*EXI_CHANNEL_SPACING)))

#define EXI_CLK_32MHZ           5

#define EXI_CSR                 0x00
#define   EXI_CSR_CLKMASK       (0x7<<4)
#define     EXI_CSR_CLK_32MHZ   (EXI_CLK_32MHZ<<4)
#define   EXI_CSR_CSMASK        (0x7<<7)
#define     EXI_CSR_CS_0        (0x1<<7)  /* Chip Select 001 */

#define EXI_CR                  0x0c
#define   EXI_CR_TSTART         (1<<0)
#define   EXI_CR_WRITE		(1<<2)
#define   EXI_CR_READ_WRITE     (2<<2)
#define   EXI_CR_TLEN(len)      (((len)-1)<<4)

#define EXI_DATA                0x10


/*
 *
 */
static int ug_check_adapter(void)
{
	u32 *csr_reg = EXI_IO_BASE(1) + EXI_CSR;
	u32 *data_reg = EXI_IO_BASE(1) + EXI_DATA;
	u32 *cr_reg = EXI_IO_BASE(1) + EXI_CR;
	u32 csr, data, cr;

	/* select */
	csr = EXI_CSR_CLK_32MHZ | EXI_CSR_CS_0;
	out_be32(csr_reg, csr);

	/* read/write */
	data = 0x90000000;
	out_be32(data_reg, data);
	cr = EXI_CR_TLEN(2) | EXI_CR_READ_WRITE | EXI_CR_TSTART;
	out_be32(cr_reg, cr);

	while(in_be32(cr_reg) & EXI_CR_TSTART)
		barrier();

	/* deselect */
	out_be32(csr_reg, 0);

	data = in_be32(data_reg);
	return (data == 0x04700000);
}

/*
 *
 */
static int ug_is_txfifo_empty(void)
{
	u32 *csr_reg = EXI_IO_BASE(1) + EXI_CSR;
	u32 *data_reg = EXI_IO_BASE(1) + EXI_DATA;
	u32 *cr_reg = EXI_IO_BASE(1) + EXI_CR;
	u32 csr, data, cr;

	/* select */
	csr = EXI_CSR_CLK_32MHZ | EXI_CSR_CS_0;
	out_be32(csr_reg, csr);

	/* read/write */
	data = 0xC0000000;
	out_be32(data_reg, data);
	cr = EXI_CR_TLEN(2) | EXI_CR_READ_WRITE | EXI_CR_TSTART;
	out_be32(cr_reg, cr);

	while(in_be32(cr_reg) & EXI_CR_TSTART)
		barrier();

	/* deselect */
	out_be32(csr_reg, 0);

	data = in_be32(data_reg);
	return (data & 0x04000000);
}

/*
 * 
 */
static void ug_putc(char ch)
{
	u32 *csr_reg = EXI_IO_BASE(1) + EXI_CSR;
	u32 *data_reg = EXI_IO_BASE(1) + EXI_DATA;
	u32 *cr_reg = EXI_IO_BASE(1) + EXI_CR;
	u32 csr, data, cr;

	/* select */
	csr = EXI_CSR_CLK_32MHZ | EXI_CSR_CS_0;
	out_be32(csr_reg, csr);

	/* write */
	data = 0xb0000000 | (ch << 20);
	out_be32(data_reg, data);
	cr = EXI_CR_TLEN(2) | EXI_CR_WRITE | EXI_CR_TSTART;
	out_be32(cr_reg, cr);

	while(in_be32(cr_reg) & EXI_CR_TSTART)
		barrier();

	/* deselect */
	out_be32(csr_reg, 0);
}

/*
 *
 */
void ug_early_putc(char ch)
{
	int tries = 10;
	while(!ug_is_txfifo_empty() && tries--)
		barrier();
	ug_putc(ch);
}


static void gamecube_console_write(const char *buf, int len)
{
	char *b = (char *)buf;

	while(len--) {
		if (*b == '\n')
			ug_early_putc('\r');
		ug_early_putc(*b++);
	}
}

#endif /* CONFIG_USBGECKO_EARLY_CONSOLE */

void platform_init(unsigned long r3, unsigned long r4, unsigned long r5)
{
	u32 heapsize = 0x01654000 - (u32)_end;

#ifdef CONFIG_USBGECKO_EARLY_CONSOLE
	if (ug_check_adapter())
		console_ops.write = gamecube_console_write;
#endif

	simple_alloc_init(_end, heapsize, 32, 64);
	ft_init(_dtb_start, 0, 4);
}

