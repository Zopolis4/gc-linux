/*
 * Datel/GameCube keyboard driver for Linux
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Should you need to contact me, the author, you can do so either by
 * e-mail - mail your message to <vojtech@ucw.cz>, or by paper mail:
 * Vojtech Pavlik, Simunkova 1594, Prague 8, 182 00 Czech Republic
 */

 /*
 There are three ways to connect a keyboard to the GameCube:

 * ASCII
   The "ASCII" keyboard is the official Nintendo/SEGA keyboard for the
   GameCube. It has 80 keys plus an Fn key; some of the keys have
   Japanese labelings. It has an LShift and an RShift key, but only a
   single Ctrl and Alt key.
   The Fn key is internal to the keyboard. It makes the keyboard send
   different scancodes if it is pressed, and an Fn keypress alone cannot
   be intercepted.

 * Datel
   The Datel Keyboard is a British IBM PS/2 keyboard that ships with an
   adapter.

 * Tototek Adapter (the Lik-Sang one)
   The Tototek adapter converts the IBM PS/2 protocol to the GameCube
   SI protocol and also converts the PS/2 scancodes into GameCube
   scancodes. The keys that have Japanese labelings on the ASCII
   keyboard get mapped to keys like PrintScreen and Pause.
   There are some interesting details about the Tototek Adapter:
   1) As there are no GameCube scancodes for RStrg and RAlt, the
      adapter sends LStrg and LAlt instead.
   2) There are GameCube scancodes both for LShift (0x54) and RShift
      (0x55), but the adapter sends LShift no matter which Shift key
      is pressed.
   3) The key right of LShift on non-US keyboards sends no scancode.
   4) The Pause/SysReq key only sends the keycode of 0x37 once, even if
      it's pressed down continuously.
   5) The Numpad is completely broken. The GameCube has no scancodes
      for the Numpad, so Tototek seems to have tried to map the PS/2
      Numpad scancodes to GameCube scancodes.
      The NumLock key sends 0x6A, a scancode that is undefined for the
      GameCube. 0-9 and Enter send the scancodes of their counterparts
      on the alphanumeric part of the keyboard, regardless of the
      status of NumLock, so he Numpad cannot be used as cursor keys.
      Numpad-- sends the same as -_, which is okay, but Numpad-+ sends
      the same as ;: - this makes sense for the the Japanese ASCII
      labeling only.
      Numpad-* sends 0x37, the same as Pause/SysReq, but this one
      continues sending it. And Numpad-/ sends 0x36, just like
      PrintScreen.
   5) All combinations of Pause/SysReq with other keys are possible,
      except for these: LStrg or LStrg together with Pause/SysReq
      doesn't send anything. This would have been SysReq.
   6) The adapter easily gets confused by two keys for which it
      produces the same GameCube scancodes: If you hold down LStrg
      and press RStrg, the LStrg scancode will disappear even though
      LStrg is srill pressed down. The same is true for Ctrl and Alt.
   It would have made sense to add a switch that disables the scancode
   conversion completely. Or, as a simple hack in the converter's
   firmware, to always send the PS/2 scancode + 0x80 as a second key
   if the scancode that is sent is ambiguous.

*/

#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/timer.h>
#include <linux/delay.h>



MODULE_AUTHOR("Steven Looman <steven@krx.nl>");
MODULE_DESCRIPTION("Datel/GameCube keyboard driver");
MODULE_LICENSE("GPL");



struct gckbd {
	unsigned long *portRegister;
	struct input_dev dev;
	struct timer_list timer;
	unsigned char lastPressed[3];
};



static unsigned char gckbd_keycode[256] = {
	/* 00 */ 0,
	/* 01 */ 0,
	/* 02 */ 0,
	/* 03 */ 0,
	/* 04 */ 0,
	/* 05 */ 0,
	/* 06 */ 102, /* TODO: Home */
	/* 07 */ 107, /* TODO: End */
	/* 08 */ 104, /* TODO: PgUp */
	/* 09 */ 109, /* TODO: PgDn */
	/* 0a */ 0x46, /* ScrollLock */
	/* 0b */ 0,
	/* 0c */ 0,
	/* 0d */ 0,
	/* 0e */ 0,
	/* 0f */ 0,
	/* 10 */ 30,   /* A */
	/* 11 */ 48,   /* B */
	/* 12 */ 46,   /* C */
	/* 13 */ 32,   /* D */
	/* 14 */ 18,   /* E */
	/* 15 */ 33,   /* F */
	/* 16 */ 34,   /* G */
	/* 17 */ 35,   /* H */
	/* 18 */ 23,   /* I */
	/* 19 */ 36,   /* J */
	/* 1a */ 37,   /* K */
	/* 1b */ 38,   /* L */
	/* 1c */ 50,   /* M */
	/* 1d */ 49,   /* N */
	/* 1e */ 24,   /* O */
	/* 1f */ 25,   /* P */
	/* 20 */ 16,   /* Q */
	/* 21 */ 19,   /* R */
	/* 22 */ 31,   /* S */
	/* 23 */ 20,   /* T */
	/* 24 */ 22,   /* U */
	/* 25 */ 47,   /* V */
	/* 26 */ 17,   /* W */
	/* 27 */ 45,   /* X */
	/* 28 */ 21,   /* Y */
	/* 29 */ 44,   /* Z */
	/* 2a */ 0x02, /* 1 */
	/* 2b */ 0x03, /* 2 */
	/* 2c */ 0x04, /* 3 */
	/* 2d */ 0x05, /* 4 */
	/* 2e */ 0x06, /* 5 */
	/* 2f */ 0x07, /* 6 */
	/* 30 */ 0x08, /* 7 */
	/* 31 */ 0x09, /* 8 */
	/* 32 */ 0x0a, /* 9 */
	/* 33 */ 0x0b, /* 0 */
	/* 34 */ 0x0c, /* -_ (or Numpad--) */
	/* 35 */ 0x0d, /* =+ */
	/* 36 */ 43, /* TODO: is either PrintScreen  or Numpad-/ */
	/* 37 */ 40, /* TODO: is either Pause/SysReq or Numpad-* */
	/* 38 */ 0x1a, /* [{ */
	/* 39 */ 0x27, /* ;: (or Numpad-+) */
	/* 3a */ 0x28, /* '" */
	/* 3b */ 0x1b, /* ]} */
	/* 3c */ 0x33, /* ,< */
	/* 3d */ 0x34, /* .> (or Numpad-.) */
	/* 3e */ 0x35, /* /? */
	/* 3f */ 0x2b, /* \| */
	/* 40 */ 0x3b, /* F1 */
	/* 41 */ 0x3c, /* F2 */
	/* 42 */ 0x3d, /* F3 */
	/* 43 */ 0x3e, /* F4 */
	/* 44 */ 0x3f, /* F5 */
	/* 45 */ 0x40, /* F6 */
	/* 46 */ 0x41, /* F7 */
	/* 47 */ 0x42, /* F8 */
	/* 48 */ 0x43, /* F9 */
	/* 49 */ 0x44, /* F10 */
	/* 4a */ 0x57, /* F11 */
	/* 4b */ 0x58, /* F12 */
	/* 4c */ 0x01, /* ESC */
	/* 4d */ 0x6e, /* TODO: Ins */
	/* 4e */ 0x6f, /* TODO: Del */
	/* 4f */ 0x29, /* `~ */
	// FIXME: 0x58 = winkey1, 0x5B = winkey2 0x5B = winkey3?
	/* 50 */ 0x0e, /* Backspace */
	/* 51 */ 0x0f, /* Tab */
	/* 52 */ 0,
	/* 53 */ 0x3a, /* CapsLock */
	/* 54 */ 0x2a, /* LShift - Tototek adapter send this code for
			  LShift and RShift */
	/* 55 */ 54,
	/* 56 */ 0x1d, /* LCtrl - Tototek adapter sends this code for
			  LCtrl and RCtrl */
	/* 57 */ 0x38, /* LAlt - Tototek adapter sends this code for
			  LAlt and RAlt */
	/* 58 */ 0,    /* TODO: LWin */
	/* 59 */ 0x39, /* Space */
	/* 5a */ 0,    /* TODO: RWin */
	/* 5b */ 0,    /* TODO: Menu */
	/* 5c */ 105,  /* TODO: Left */
	/* 5d */ 108,  /* TODO: Down */
	/* 5e */ 103,  /* TODO: Up */
	/* 5f */ 106,  /* TODO: Right */
	/* 60 */ 0,
	/* 61 */ 28,   /* Enter */
	/* 62 */ 0,
	/* 63 */ 0,
	/* 64 */ 39,
	/* 65 */ 78,
	/* 66 */ 0,
	/* 67 */ 0,
	/* 68 */ 0,
	/* 69 */ 0,
	/* 6a */ 0x45, /* NumLock */
	/* 6b */ 0,
	/* 6c */ 0,
	/* 6d */ 0,
	/* 6e */ 0,
	/* 6f */ 0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 69,  0,  0, 70,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};
static struct resource gckbd_resource = {
	"gckbd",
	0xCC006400,
	0xCC006440,
	IORESOURCE_MEM|IORESOURCE_BUSY
};



static void gckbd_timer(unsigned long private)
{
	// this could probably be done with an array
	unsigned char key1, key2, key3;
	int i;
	struct gckbd *kbd = (struct gckbd *)private;


	// read keys pressed
	key1 = (*kbd->portRegister >> 24) & 0xFF;
	key2 = (*kbd->portRegister >> 16) & 0xFF;
	key3 = (*kbd->portRegister >>  8) & 0xFF;


//	printk("-%x %x %x - %x %x %x-\n", key1, key2, key3, kbd->lastPressed[0],
//			kbd->lastPressed[1], kbd->lastPressed[2]);


	// report released keys
	for (i = 0; i < 3; i++) {
		if (kbd->lastPressed[i]) {
			if (kbd->lastPressed[i] != key1 &&
			    kbd->lastPressed[i] != key2 &&
			    kbd->lastPressed[i] != key3) {
//				printk("released%d: %x\n", i, kbd->lastPressed[i]);
				input_report_key (&kbd->dev, gckbd_keycode[kbd->lastPressed[i]], 0);
			}
		}
	}


	// check if we should report anything
	if (key1) {
//		printk("pressed1: %x\n", key1);
		input_report_key (&kbd->dev, gckbd_keycode[key1], 1);
	}
	if (key2) {
//		printk("pressed2: %x\n", key2);
		input_report_key (&kbd->dev, gckbd_keycode[key2], 1);
	}
	if (key3) {
//		printk("pressed3: %x\n", key3);
		input_report_key (&kbd->dev, gckbd_keycode[key3], 1);
	}

	input_sync (&kbd->dev);


	// keep state of pressed keys
	kbd->lastPressed[0] = key1;
	kbd->lastPressed[1] = key2;
	kbd->lastPressed[2] = key3;


	mod_timer (&kbd->timer, jiffies + HZ/50);
}



static int gckbd_open(struct input_dev *dev)
{
	struct gckbd *kbd = dev->private;


	init_timer (&kbd->timer);
	kbd->timer.data = (unsigned int)kbd;
	kbd->timer.function = gckbd_timer;
	kbd->timer.expires = jiffies + HZ/50;
	add_timer (&kbd->timer);


	return 0;
}



static void gckbd_close(struct input_dev *dev)
{
	struct gckbd *kbd = dev->private;


	del_timer (&kbd->timer);
}



static int __init gckbd_init(void)
{
	int i;
	struct gckbd *kbd;


	if (request_resource (&iomem_resource, &gckbd_resource) < 0) {
		printk(KERN_WARNING "gcpad : resource busy\n");
		return -EBUSY;
	}


	kbd = kmalloc (sizeof (struct gckbd), GFP_KERNEL);
	if (kbd == NULL)
		return -ENOMEM;

	memset (kbd, 0, sizeof (struct gckbd));


	// set some parameters/callbacks
	kbd->portRegister = (unsigned long*)0xCC00642C;
	kbd->dev.private = kbd;
	kbd->dev.open = gckbd_open;
	kbd->dev.close = gckbd_close;

	set_bit (EV_KEY, kbd->dev.evbit);
	set_bit (EV_REP, kbd->dev.evbit);
	for (i = 0; i < 255; i++)
		set_bit (gckbd_keycode[i], kbd->dev.keybit);

	// register the device
	input_register_device (&kbd->dev);


	printk (KERN_INFO "gamecube keyboard: started\n");


	return 0;
}



static void __exit gckbd_exit(void)
{
	// this is a BAD thing(tm), not releasing stuff.. this will give troubles ;)
/*
	input_unregister_device (&kbd->dev);

	kfree (gckbd);
*/	
	release_resource (&gckbd_resource);
}



module_init (gckbd_init);
module_exit (gckbd_exit);
