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
	unsigned char pressedKeys[3];
};



static unsigned char gckbd_keycode[256] = {
	 0,  0,  0,  0,  0,  0,102,107,104,109,  0,  0,  0,  0,  0,  0,
	30, 48, 46, 32, 18, 33, 34, 35, 23, 36, 37, 38, 50, 49, 24, 25,
	16, 19, 31, 20, 22, 47, 17, 45, 21, 44,  2,  3,  4,  5,  6,  7,
	 8,  9, 10, 11, 12,  0, 43, 40, 26, 13, 55, 27, 51, 52, 53,  0,
	59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 87, 88,  1,110,111, 41,
	// FIXME: 0x58 = winkey1, 0x5B = winkey2 0x5B = winkey3?
	14, 15,  0, 58, 42, 54, 29, 56,  0, 57,  0,  0,105,108,103,106,
	 0, 28,  0,  0, 39, 78,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
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


	// report released keys
	for (i = 0; i < 3; i++)
	{
		if (kbd->pressedKeys[i] && kbd->pressedKeys[i] != key1 && kbd->pressedKeys[i] != key2 && kbd->pressedKeys[i] != key3)
			input_report_key (&kbd->dev, gckbd_keycode[kbd->pressedKeys[i]], 0);
	}



	// check if we should report anything
	if (key1 && gckbd_keycode[key1])
		input_report_key (&kbd->dev, gckbd_keycode[key1], 1);
	if (key2 && gckbd_keycode[key2])
		input_report_key (&kbd->dev, gckbd_keycode[key2], 1);
	if (key3 && gckbd_keycode[key3])
		input_report_key (&kbd->dev, gckbd_keycode[key3], 1);

	input_sync (&kbd->dev);


	// keep state of pressed keys
	kbd->pressedKeys[0] = key1;
	kbd->pressedKeys[1] = key1;
	kbd->pressedKeys[2] = key1;


	mod_timer (&kbd->timer, jiffies + HZ/50);
}



static int gckbd_open(struct input_dev *dev)
{
	struct gckbd *kbd = dev->private;


	init_timer (&kbd->timer);
	kbd->timer.data = (void *)kbd;
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
