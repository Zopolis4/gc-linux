/*
 * arch/ppc/platforms/gamecube_pic.c
 */

#undef DEBUG


#include <linux/irq.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <asm/io.h>


static void gamecube_mask_and_ack_irq(unsigned int irq)
{
	pr_debug("mask_and_ack(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM),
			GAMECUBE_IN(GAMECUBE_PIIC));
	if (irq < GAMECUBE_IRQS) {
		GAMECUBE_OUT(GAMECUBE_PIIM, GAMECUBE_IN(GAMECUBE_PIIM) &
				~(1 << irq)); /* mask */
		GAMECUBE_OUT(GAMECUBE_PIIC, 1 << irq); /* ack */
	}
	pr_debug("after mask_and_ack(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM),
			GAMECUBE_IN(GAMECUBE_PIIC));
}

static void gamecube_mask_irq(unsigned int irq)
{
	pr_debug("mask(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM),
			GAMECUBE_IN(GAMECUBE_PIIC));
	if (irq < GAMECUBE_IRQS)
		GAMECUBE_OUT(GAMECUBE_PIIM, GAMECUBE_IN(GAMECUBE_PIIM) &
				~(1 << irq)); /* mask */
}

static void
gamecube_unmask_irq(unsigned int irq)
{
	pr_debug(" before unmask(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM),
			GAMECUBE_IN(GAMECUBE_PIIC));
	if (irq < GAMECUBE_IRQS)
		GAMECUBE_OUT(GAMECUBE_PIIM, GAMECUBE_IN(GAMECUBE_PIIM) |
				(1 << irq));
	pr_debug("after unmask(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM),
			GAMECUBE_IN(GAMECUBE_PIIC));
}


static struct hw_interrupt_type gamecube_pic = {
	.typename	= " GC-PIC ",
	.enable		= gamecube_unmask_irq,
	.disable	= gamecube_mask_irq,
	.ack		= gamecube_mask_and_ack_irq,
};

void __init gamecube_init_IRQ(void)
{
	int i;

	GAMECUBE_OUT(GAMECUBE_PIIM, 0); /* disable all irqs */
	GAMECUBE_OUT(GAMECUBE_PIIC, 0xffffffff); /* ack all irqs */

	for (i = 0; i < GAMECUBE_IRQS; i++)
		irq_desc[i].handler = &gamecube_pic;
}

/*
 * Find the highest IRQ that's generating an interrupt, if any.
 */
int gamecube_get_irq(struct pt_regs *regs)
{
	int irq = 0;
	u_int irq_status, irq_test = 1;

	pr_debug("get_irq(): %x, %x\n", GAMECUBE_IN(GAMECUBE_PIIM),
			GAMECUBE_IN(GAMECUBE_PIIC));
	irq_status = GAMECUBE_IN(GAMECUBE_PIIC) & GAMECUBE_IN(GAMECUBE_PIIM);

	if (irq_status == 0) {
		pr_debug("GC-PIC: Received a spurious IRQ\n");
		return -1;
/*		while(1);*/
	}

	do {
		if (irq_status & irq_test)
			break;
		irq++;
		irq_test <<= 1;
	} while (irq < GAMECUBE_IRQS);

	return irq;
}
