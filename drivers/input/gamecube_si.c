#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/timer.h>
#include <linux/delay.h>

#include <asm/io.h>

// this keymap is for a datel adapter + normal US keyboard, if you need somehitng else
// copy the keymap, edit it and change the #include
#include "gamecube_keymap.h"



MODULE_AUTHOR ("Steven Looman <steven@krx.nl>");
MODULE_DESCRIPTION ("Nintendo Gamecube controller input driver");
MODULE_LICENSE ("GPL");



#define REFRESH_TIME HZ/100

#define SICOUTBUF(x)	(0xCC006400 + x * 12)
#define SICINBUFH(x)	(0xCC006404 + x * 12)
#define SICINBUFL(x)	(0xCC006408 + x * 12)

#define SIPOLL		0xCC006430
#define SICOMCSR	0xCC006434
#define SISR		0xCC006438
#define SIEXILK		0xCC00643C


#define ID_PAD		0x0900
#define ID_KEYBOARD	0x0820
#define ID_GBA		0x0004
#define ID_GBA_NA	0x0800
#define ID_WAVEBIRD	0xa800
#define ID_WAVEBIRD_RCV	0xe960

#define PAD_START	(1 << 28)
#define PAD_Y		(1 << 27)
#define PAD_X		(1 << 26)
#define PAD_B		(1 << 25)
#define PAD_A		(1 << 24)
#define PAD_LT		(1 << 22)
#define PAD_RT		(1 << 21)
#define PAD_Z		(1 << 20)
#define PAD_UP		(1 << 19)
#define PAD_DOWN	(1 << 18)
#define PAD_RIGHT	(1 << 17)
#define PAD_LEFT	(1 << 16)

//#define GCSIDEBUG



static struct resource resources = {
	"Gamecube SI",
	0xCC006400,
	0xCC006500,
	IORESOURCE_MEM|IORESOURCE_BUSY
};



typedef struct {
	unsigned char old[3];
} keyboardStatus;

struct {
	unsigned int id;
	unsigned int raw[2];

	unsigned char errStat : 1;
	unsigned char errLatch : 1;

	struct input_dev inputDev;
	struct timer_list timer;

	union {
		keyboardStatus keyboard;
	};

	char name[32];
//	char phys[32];
} port[4];



static void resetSi (void) {
	int i;
	// clear si registers
	// SICOUTBUF
	for (i = 0; i < 4; ++i)
		writel (0, SICOUTBUF (i));

	// SICINBUFH
	for (i = 0; i < 4; ++i)
		writel (0, SICINBUFH (i));

	// SICINBUFL
	for (i = 0; i < 4; ++i)
		writel (0, SICINBUFL (i));

	writel (0, SIPOLL);
	writel (0, SICOMCSR);
	writel (0, SISR);

	writel (0, 0xCC006480);
	writel (0, 0xCC006484);
	writel (0, 0xCC006488);
	writel (0, 0xCC00648c);

	writel (0, 0xCC006490);
	writel (0, 0xCC006494);
	writel (0, 0xCC006498);
	writel (0, 0xCC00649c);

	writel (0, 0xCC0064a0);
	writel (0, 0xCC0064a4);
	writel (0, 0xCC0064a8);
	writel (0, 0xCC0064ac);
}


static void waitTransferDone (void) {
	unsigned long transferDone;

	do {
		transferDone = readl (SICOMCSR) & (1 << 31);
	} while (!transferDone);

	writel (readl (SICOMCSR) |= (1 << 31), SICOMCSR); // mask IRQ
}


static unsigned long getControllerID (int port) {
	resetSi();

	writel (0, SIPOLL);
	writel (0, SICOUTBUF (port));
	writel (0x80000000, SISR);
	writel (0xD0010001 | port << 1, SICOMCSR);

	waitTransferDone();

	return readl (0xCC006480);
}


static void setPolling (void) {
	unsigned long padBits = 0;
	int i;

	for (i = 0; i < 4; ++i) {
		switch (port[i].id) {
			case ID_PAD:
				writel (0x00400300, SICOUTBUF (i));
				break;

			case ID_KEYBOARD:
				writel (0x00540000, SICOUTBUF (i));
				break;
		}

		padBits |= 1 << (7 - i);
	}

	writel (0x00F70200 | padBits, SIPOLL);
	writel (0x80000000, SISR);
	writel (0xC0010801, SICOMCSR);

	waitTransferDone();
}



static void setRumbling (unsigned char portNo, unsigned char rumble) {
	if (rumble) {
		writel (0x00400001, SICOUTBUF (portNo));
		writel (0x80000000, SISR);
	} else {
		writel (0x00400000, SICOUTBUF (portNo));
		writel (0x80000000, SISR);
	}
}



static void gcSiTimer (unsigned long portNo) {
	int i;
	unsigned long raw[2];
	unsigned char key[3];

	raw[0] = readl (SICINBUFH (portNo));
	raw[1] = readl (SICINBUFL (portNo));
	
	switch (port[portNo].id) {
		case ID_PAD:
			// buttons
			input_report_key (&port[portNo].inputDev, BTN_A, raw[0] & PAD_A);
			input_report_key (&port[portNo].inputDev, BTN_B, raw[0] & PAD_B);
			input_report_key (&port[portNo].inputDev, BTN_X, raw[0] & PAD_X);
			input_report_key (&port[portNo].inputDev, BTN_Y, raw[0] & PAD_Y);
			input_report_key (&port[portNo].inputDev, BTN_Z, raw[0] & PAD_Z);
			input_report_key (&port[portNo].inputDev, BTN_TL, raw[0] & PAD_LT);
			input_report_key (&port[portNo].inputDev, BTN_TR, raw[0] & PAD_RT);
			input_report_key (&port[portNo].inputDev, BTN_START, raw[0] & PAD_START);

			// axis
			// a stick
			input_report_abs (&port[portNo].inputDev, ABS_X, (raw[0] >>  8) & 0xFF);
			input_report_abs (&port[portNo].inputDev, ABS_Y, -(raw[0] >>  0) & 0xFF);

			// b pad
			if (raw[0] & PAD_RIGHT)
				input_report_abs (&port[portNo].inputDev, ABS_HAT0X, 1);
			else if (raw[0] & PAD_LEFT)
				input_report_abs (&port[portNo].inputDev, ABS_HAT0X, -1);
			else
				input_report_abs (&port[portNo].inputDev, ABS_HAT0X, 0);
			
			if (raw[0] & PAD_UP)
				input_report_abs (&port[portNo].inputDev, ABS_HAT0Y, 1);
			else if (raw[0] & PAD_DOWN)
				input_report_abs (&port[portNo].inputDev, ABS_HAT0Y, -1);
			else
				input_report_abs (&port[portNo].inputDev, ABS_HAT0Y, 0);
			
			// c stick
			input_report_abs (&port[portNo].inputDev, ABS_RX, raw[1] >> 24 & 0xFF);
			input_report_abs (&port[portNo].inputDev, ABS_RY, raw[1] >> 16 & 0xFF);

			// triggers
			input_report_abs (&port[portNo].inputDev, ABS_BRAKE, raw[1] >> 8 & 0xFF);
			input_report_abs (&port[portNo].inputDev, ABS_GAS,   raw[1] >> 0 & 0xFF);

			break;

		case ID_KEYBOARD:
			key[0] = (raw[0] >> 12) & 0xFF;
			key[1] = (raw[0] >>  4) & 0xFF;
			key[2] = (raw[0] <<  4) & 0xFF;
			key[2]|= (raw[1] << 28) & 0xFF;

			// check if anything was released
			for (i = 0; i < 3; ++i) {
				unsigned char oldKey = port[portNo].keyboard.old[i];
				if (oldKey != key[0] &&
					oldKey != key[1] &&
					oldKey != key[2])
					input_report_key (&port[portNo].inputDev, gamecube_keymap[oldKey], 0);
			}

			// reports keys
			for (i = 0; i < 3; ++i) {
				if (key[i])
					input_report_key (&port[portNo].inputDev, gamecube_keymap[key[i]], 1);

				port[portNo].keyboard.old[i] = key[i];
			}

			break;

		default:
			break;
	}

	input_sync(&port[portNo].inputDev);

	mod_timer (&port[portNo].timer, jiffies + REFRESH_TIME);
}



static int gcSiOpen (struct input_dev *inputDev) {
	int portNo = (int)inputDev->private;
	
	init_timer (&port[portNo].timer);
	port[portNo].timer.function = gcSiTimer;
	port[portNo].timer.data     = (int)inputDev->private;
	port[portNo].timer.expires  = jiffies + REFRESH_TIME;
	add_timer (&port[portNo].timer);

	return 0;
}



static void gcSiClose (struct input_dev *inputDev) {
	int portNo = (int)inputDev->private;

	del_timer (&port[portNo].timer);
}



static int gcSiEvent (struct input_dev *dev, unsigned int type, unsigned int code, int value) {
	int portNo = (int)dev->private;
	
	if (type == EV_FF)
		if (code == FF_RUMBLE)
			setRumbling (portNo, value);
	
	return value;
}



static int __init gcSiInit(void) {
	int i;

	printk (KERN_WARNING "Gamecube SI: init\n");

	if (request_resource (&iomem_resource, &resources) < 0) {
		printk (KERN_WARNING "Gamecube SI: resource busy\n");
		return -EBUSY;
	}

	for (i = 0; i < 4; ++i) {
		int j;

		memset (&port[i], 0, sizeof (port[i]));

		// probe ports
		port[i].id = getControllerID(i) >> 16;
#ifdef GCSIDEBUG
		printk (KERN_WARNING "Gamecube SI: port[%d] = 0x%x\n", i, port[i].id);
#endif

		init_input_dev (&port[i].inputDev);

		port[i].inputDev.open = gcSiOpen;
		port[i].inputDev.close = gcSiClose;
		port[i].inputDev.private = (unsigned int *)i;

		printk (KERN_WARNING "Gamecube Si: Port %d: ", i);

		switch (port[i].id) {
			case ID_PAD:
				printk (KERN_WARNING "standard pad\n");
				
				sprintf (port[i].name, "Gamecube standard pad");
				port[i].inputDev.name = port[i].name;

//				sprintf (port[i].phys, "gcsi/port%d", i);
//				port[i].inputDev.phys = port[i].phys;

				set_bit (EV_KEY, port[i].inputDev.evbit);
				set_bit (EV_ABS, port[i].inputDev.evbit);
				set_bit (EV_FF,  port[i].inputDev.evbit);

				set_bit (BTN_A, port[i].inputDev.keybit);
				set_bit (BTN_B, port[i].inputDev.keybit);
				set_bit (BTN_X, port[i].inputDev.keybit);
				set_bit (BTN_Y, port[i].inputDev.keybit);
				set_bit (BTN_Z, port[i].inputDev.keybit);
				set_bit (BTN_TL, port[i].inputDev.keybit);
				set_bit (BTN_TR, port[i].inputDev.keybit);
				set_bit (BTN_START, port[i].inputDev.keybit);
				
				// a stick
				set_bit (ABS_X, port[i].inputDev.absbit);
				port[i].inputDev.absmin[ABS_X] = 0;
				port[i].inputDev.absmax[ABS_X] = 255;
				port[i].inputDev.absfuzz[ABS_X] = 8;
				port[i].inputDev.absflat[ABS_X] = 8;
				
				set_bit (ABS_Y, port[i].inputDev.absbit);
				port[i].inputDev.absmin[ABS_Y] = 0;
				port[i].inputDev.absmax[ABS_Y] = 255;
				port[i].inputDev.absfuzz[ABS_Y] = 8;
				port[i].inputDev.absflat[ABS_Y] = 8;
				
				// b pad
				set_bit (ABS_HAT0X, port[i].inputDev.absbit);
				port[i].inputDev.absmin[ABS_HAT0X] = -1;
				port[i].inputDev.absmax[ABS_HAT0X] = 1;

				set_bit (ABS_HAT0Y, port[i].inputDev.absbit);
				port[i].inputDev.absmin[ABS_HAT0Y] = -1;
				port[i].inputDev.absmax[ABS_HAT0Y] = 1;
				
				// c stick
				set_bit (ABS_RX, port[i].inputDev.absbit);
				port[i].inputDev.absmin[ABS_RX] = 0;
				port[i].inputDev.absmax[ABS_RX] = 255;
				port[i].inputDev.absfuzz[ABS_RX] = 8;
				port[i].inputDev.absflat[ABS_RX] = 8;
				
				set_bit (ABS_RY, port[i].inputDev.absbit);
				port[i].inputDev.absmin[ABS_RY] = 0;
				port[i].inputDev.absmax[ABS_RY] = 255;
				port[i].inputDev.absfuzz[ABS_RY] = 8;
				port[i].inputDev.absflat[ABS_RY] = 8;
				
				// triggers
				set_bit (ABS_GAS, port[i].inputDev.absbit);
				port[i].inputDev.absmin[ABS_GAS] = -255;
				port[i].inputDev.absmax[ABS_GAS] = 255;
				port[i].inputDev.absfuzz[ABS_GAS] = 16;
				port[i].inputDev.absflat[ABS_GAS] = 16;
				
				set_bit (ABS_BRAKE, port[i].inputDev.absbit);
				port[i].inputDev.absmin[ABS_BRAKE] = -255;
				port[i].inputDev.absmax[ABS_BRAKE] = 255;
				port[i].inputDev.absfuzz[ABS_BRAKE] = 16;
				port[i].inputDev.absflat[ABS_BRAKE] = 16;
				
				// rumbling
				set_bit (FF_RUMBLE, port[i].inputDev.ffbit);
				port[i].inputDev.event = gcSiEvent;
				
				port[i].inputDev.ff_effects_max = 1;

				input_register_device (&port[i].inputDev);

				break;

			case ID_KEYBOARD:
				printk (KERN_WARNING "keyboard\n");

				set_bit (EV_KEY, port[i].inputDev.evbit);
				set_bit (EV_REP, port[i].inputDev.evbit);

				for (j = 0; j < 255; ++j)
					set_bit (gamecube_keymap[j], port[i].inputDev.keybit);

				input_register_device (&port[i].inputDev);

				break;

			default:
				// unknown device
				if (port[i].id)
					printk (KERN_WARNING "unknown device (%x)\n", port[i].id);
				else
					printk (KERN_WARNING "no device\n");
					
				break;
		}
	}

	setPolling();

	return 0;
}


static void __exit gcSiExit(void) {
	int i;
	
	printk (KERN_WARNING "Gamecube SI: exit\n");

	for (i = 0; i < 4; ++i)
		if (port[i].id == ID_PAD ||
		    port[i].id == ID_KEYBOARD)
			input_unregister_device (&port[i].inputDev);
	
	release_resource (&resources);
}



module_init (gcSiInit);
module_exit (gcSiExit);
