/*
 * arch/ppc/platforms/gamecube_pic.c
 */

#include <linux/irq.h>
#include <linux/init.h>

#include <asm/io.h>
#include <asm/bitops.h>

#include "gamecube.h"


static void gekko_mask_and_ack_irq(unsigned int irq)
{
	clear_bit(irq, GAMECUBE_PIIM);
	set_bit(irq, GAMECUBE_PIIC);
}

static void gekko_mask_irq(unsigned int irq)
{
	clear_bit(irq, GAMECUBE_PIIM);
}

static void gekko_unmask_irq(unsigned int irq)
{
	set_bit(irq, GAMECUBE_PIIM);
}

static struct hw_interrupt_type gekko_pic = {
	.typename	= " GEKKO-PIC ",
	.enable		= gekko_unmask_irq,
	.disable	= gekko_mask_irq,
	.ack		= gekko_mask_and_ack_irq,
};

void __init gamecube_init_IRQ(void)
{
	int i;

	/* mask and ack all IRQs */
	writel(0x00000000, GAMECUBE_PIIM);
	writel(0xffffffff, GAMECUBE_PIIC);

	for (i = 0; i < GAMECUBE_IRQS; i++)
		irq_desc[i].handler = &gekko_pic;
}

/*
 * Find the highest IRQ that's generating an interrupt, if any.
 */
int gamecube_get_irq(struct pt_regs *regs)
{
	int irq;
	u_int irq_status, irq_test = 1;

	irq_status = readl(GAMECUBE_PIIC) & readl(GAMECUBE_PIIM);
	if (irq_status == 0)
		return -1;	/* no more IRQs pending */

	for (irq = 0; irq < GAMECUBE_IRQS; irq++, irq_test <<= 1)
		if (irq_status & irq_test)
			break;

	return irq;
}
