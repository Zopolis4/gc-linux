/*
 * arch/powerpc/platforms/embedded6xx/ugecon.c
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

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/console.h>

#include <asm/processor.h>

#include "gamecube.h"


#define EXI_BASE		(GCN_IO2_BASE+0x6800)
#define EXI_CHANNEL_SPACING	0x14

#define EXI_IO_BASE(c)	((void __iomem *)(EXI_BASE + ((c)*EXI_CHANNEL_SPACING)))

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
        u32 __iomem *csr_reg = EXI_IO_BASE(1) + EXI_CSR;
        u32 __iomem *data_reg = EXI_IO_BASE(1) + EXI_DATA;
        u32 __iomem *cr_reg = EXI_IO_BASE(1) + EXI_CR;
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
        u32 __iomem *csr_reg = EXI_IO_BASE(1) + EXI_CSR;
        u32 __iomem *data_reg = EXI_IO_BASE(1) + EXI_DATA;
        u32 __iomem *cr_reg = EXI_IO_BASE(1) + EXI_CR;
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
        u32 __iomem *csr_reg = EXI_IO_BASE(1) + EXI_CSR;
        u32 __iomem *data_reg = EXI_IO_BASE(1) + EXI_DATA;
        u32 __iomem *cr_reg = EXI_IO_BASE(1) + EXI_CR;
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
	int count = 10;

        while(!ug_is_txfifo_empty() && count--) 
                barrier();
        ug_putc(ch);
}

/*
 *
 */
void ug_early_puts(char *s)
{
	while(*s) {
		if (*s == '\n')
			ug_early_putc('\r');
		ug_early_putc(*s++);
	}
}

/*
 *
 */
static void ug_early_con_write(struct console *co, const char *buf,
				   unsigned int count)
{
	char *b = (char *)buf;
	while (count--) {
		if (*b == '\n')
			ug_early_putc('\r');
		ug_early_putc(*b++);
	}
}


static struct console ug_early_con = {
	.name = "ugecon",
	.flags = CON_PRINTBUFFER | CON_BOOT,
	.index = -1,
};

/*
 *
 */
void ug_early_con_init(void)
{
	if (ug_check_adapter()) {
		ug_early_puts("ugecon: early console initialized.\n");

		ug_early_con.write = ug_early_con_write;
		register_console(&ug_early_con);
	}
}

