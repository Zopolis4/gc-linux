#ifndef __exi_h
#define __exi_h

void exi_select(int channel, int device, int freq);
void exi_deselect(int channel);

#define EXI_READ  0
#define EXI_WRITE 1

void exi_imm(int channel, void *data, int len, int mode, int zero);
void exi_sync(int channel);
void exi_imm_ex(int channel, void *data, int len, int mode);

void exi_init(void);

#define EXI_EVENT_IRQ     0
#define EXI_EVENT_INSERT  1
#define EXI_EVENT_TC      2

typedef void (exi_irq_handler_t)(int channel, int event, void *context);
int exi_request_irq(int channel, int event, exi_irq_handler_t *handler, void *context);
int exi_free_irq(int channel, int event);

#endif

