/*
 * drivers/video/gcnfb.c
 *
 * Nintendo GameCube "Flipper" chipset frame buffer driver
 * Copyright (C) 2004 Michael Steil <mist@c64.org>
 * Copyright (C) 2004 The GameCube Linux Team
 *
 * Based on vesafb (c) 1998 Gerd Knorr <kraxel@goldbach.in-berlin.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <asm/io.h>

#include <platforms/gamecube.h>


#define DRV_MODULE_NAME   "gcnfb"
#define DRV_DESCRIPTION   "Nintendo GameCube frame buffer driver"
#define DRV_AUTHOR        "Michael Steil <mist@c64.org>"

MODULE_AUTHOR(DRV_AUTHOR);
MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_LICENSE(GPL);


volatile static unsigned int *gcn_video = (unsigned int *)0xCC002000;

static const unsigned int VIDEO_Mode640X480NtscYUV16[32] = {
	0x0F060001, 0x476901AD, 0x02EA5140, 0x00030018,
	0x00020019, 0x410C410C, 0x40ED40ED, 0x00435A4E,
	0x00000000, 0x00435A4E, 0x00000000, 0x00000000,
	0x110701AE, 0x10010001, 0x00010001, 0x00010001,
	0x00000000, 0x00000000, 0x28500100, 0x1AE771F0,
	0x0DB4A574, 0x00C1188E, 0xC4C0CBE2, 0xFCECDECF,
	0x13130F08, 0x00080C0F, 0x00FF0000, 0x00000000,
	0x02800000, 0x000000FF, 0x00FF00FF, 0x00FF00FF
};

static const unsigned int VIDEO_Mode640X480Pal50YUV16[32] = {
	0x11F50101, 0x4B6A01B0, 0x02F85640, 0x00010023,
	0x00000024, 0x4D2B4D6D, 0x4D8A4D4C, 0x00435A4E,
	0x00000000, 0x00435A4E, 0x00000000, 0x013C0144,
	0x113901B1, 0x10010001, 0x00010001, 0x00010001,
	0x00000000, 0x00000000, 0x28500100, 0x1AE771F0,
	0x0DB4A574, 0x00C1188E, 0xC4C0CBE2, 0xFCECDECF,
	0x13130F08, 0x00080C0F, 0x00FF0000, 0x00000000,
	0x02800000, 0x000000FF, 0x00FF00FF, 0x00FF00FF
};

static struct fb_var_screeninfo gcnfb_defined __initdata = {
	.activate = FB_ACTIVATE_NOW,
	.height = -1,
	.width = -1,
	.right_margin = 32,
	.upper_margin = 16,
	.lower_margin = 4,
	.vsync_len = 4,
	.vmode = FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo gcnfb_fix __initdata = {
	.id = "gcnfb",
	.type = FB_TYPE_PACKED_PIXELS,
	.accel = FB_ACCEL_NONE,
};

#define TV_ENC_DETECT 0
#define TV_ENC_NTSC 1
#define TV_ENC_PAL 2
static int tv_encoding __initdata = TV_ENC_DETECT;

static struct fb_info gcnfb_info;
static u32 pseudo_palette[17];

static int ypan = 0;		/* 0..nothing, 1..ypan, 2..ywrap */

/*
 * RGB to YCbYCr conversion support bits.
 * We are using here the ITU.BT-601 Y'CbCr standard.
 *
 * References:
 * - "Colour Space Conversions" by Adrian Ford and Alan Roberts, 1998
 *   (google for coloureq.pdf)
 *
 */

#define RGB2YUV_SHIFT   16
#define RGB2YUV_LUMA    16
#define RGB2YUV_CHROMA 128

#define Yr ((int)( 0.299*(1<<RGB2YUV_SHIFT)))
#define Yg ((int)( 0.587*(1<<RGB2YUV_SHIFT)))
#define Yb ((int)( 0.114*(1<<RGB2YUV_SHIFT)))

#define Ur ((int)(-0.169*(1<<RGB2YUV_SHIFT)))
#define Ug ((int)(-0.331*(1<<RGB2YUV_SHIFT)))
#define Ub ((int)( 0.500*(1<<RGB2YUV_SHIFT)))

#define Vr ((int)( 0.500*(1<<RGB2YUV_SHIFT)))	/* same as Ub */
#define Vg ((int)(-0.419*(1<<RGB2YUV_SHIFT)))
#define Vb ((int)(-0.081*(1<<RGB2YUV_SHIFT)))

#define clamp(x, y, z) ((z < x) ? x : ((z > y) ? y : z))

static inline uint32_t rgbrgb16toycbycr(uint16_t rgb1, uint16_t rgb2)
{
	register int Y1, Cb, Y2, Cr;
	register int r1, g1, b1;
	register int r2, g2, b2;
	register int r, g, b;

	/* fast path, thanks to bohdy */
	if (!(rgb1 | rgb2)) {
		return 0x00800080;	/* black, black */
	}

	/* RGB565 */
	r1 = ((rgb1 >> 11) & 0x1f);
	g1 = ((rgb1 >> 5) & 0x3f);
	b1 = ((rgb1 >> 0) & 0x1f);

	/* fast (approximated) scaling to 8 bits, thanks to Masken */
	r1 = (r1 << 3) | (r1 >> 2);
	g1 = (g1 << 2) | (g1 >> 4);
	b1 = (b1 << 3) | (b1 >> 2);

	Y1 = clamp(16, 235, ((Yr * r1 + Yg * g1 + Yb * b1) >> RGB2YUV_SHIFT)
		   + RGB2YUV_LUMA);
	if (rgb1 == rgb2) {
		/* this is just another fast path */
		Y2 = Y1;
		r = r1;
		g = g1;
		b = b1;
	} else {
		/* same as we did for r1 before */
		r2 = ((rgb2 >> 11) & 0x1f);
		g2 = ((rgb2 >> 5) & 0x3f);
		b2 = ((rgb2 >> 0) & 0x1f);
		r2 = (r2 << 3) | (r2 >> 2);
		g2 = (g2 << 2) | (g2 >> 4);
		b2 = (b2 << 3) | (b2 >> 2);

		Y2 = clamp(16, 235,
			   ((Yr * r2 + Yg * g2 + Yb * b2) >> RGB2YUV_SHIFT)
			   + RGB2YUV_LUMA);

		r = (r1 + r2) / 2;
		g = (g1 + g2) / 2;
		b = (b1 + b2) / 2;
	}

	Cb = clamp(16, 240, ((Ur * r + Ug * g + Ub * b) >> RGB2YUV_SHIFT)
		   + RGB2YUV_CHROMA);
	Cr = clamp(16, 240, ((Vr * r + Vg * g + Vb * b) >> RGB2YUV_SHIFT)
		   + RGB2YUV_CHROMA);

	return (((uint8_t) Y1) << 24) | (((uint8_t) Cb) << 16) |
	    (((uint8_t) Y2) << 8) | (((uint8_t) Cr) << 0);
}

unsigned int gcnfb_writel(unsigned int rgbrgb, void *address)
{
	uint16_t *rgb = (uint16_t *) & rgbrgb;
	return fb_writel_real(rgbrgb16toycbycr(rgb[0], rgb[1]), address);
}

static int gcnfb_pan_display(struct fb_var_screeninfo *var,
				  struct fb_info *info)
{
	return 0;
}

static void vesa_setpalette(int regno, unsigned red, unsigned green,
			    unsigned blue)
{
}

static int gcnfb_setcolreg(unsigned regno, unsigned red, unsigned green,
				unsigned blue, unsigned transp,
				struct fb_info *info)
{
	/*
	 *  Set a single color register. The values supplied are
	 *  already rounded down to the hardware's capabilities
	 *  (according to the entries in the `var' structure). Return
	 *  != 0 for invalid regno.
	 */

	if (regno >= info->cmap.len)
		return 1;

	switch (info->var.bits_per_pixel) {
	case 8:
		vesa_setpalette(regno, red, green, blue);
		break;
	case 15:
	case 16:
		if (info->var.red.offset == 10) {
			/* XXX, not used currently */
			/* 1:5:5:5 */
			((u32 *) (info->pseudo_palette))[regno] =
			    ((red & 0xf800) >> 1) |
			    ((green & 0xf800) >> 6) | ((blue & 0xf800) >> 11);
		} else {
			/* 0:5:6:5 */
			((u32 *) (info->pseudo_palette))[regno] =
			    ((red & 0xf800)) |
			    ((green & 0xfc00) >> 5) | ((blue & 0xf800) >> 11);
		}
		break;
	case 24:
	case 32:
		/* XXX, not used currently */
		red >>= 8;
		green >>= 8;
		blue >>= 8;
		((u32 *) (info->pseudo_palette))[regno] =
		    (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset);
		break;
	}
	return 0;
}

static struct fb_ops gcnfb_ops = {
	.owner = THIS_MODULE,
	.fb_setcolreg = gcnfb_setcolreg,
	.fb_pan_display = gcnfb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = soft_cursor,
};

int __init gcnfb_setup(char *options)
{
	char *this_opt;

	if (!options || !*options)
		return 0;

	printk("gcnfb: options = %s\n", options);

	while ((this_opt = strsep(&options, ",")) != NULL) {
		printk("this_opt = %s\n", this_opt);
		if (!*this_opt)
			continue;

		if (!strcmp(this_opt, "redraw"))
			ypan = 0;
		else if (!strcmp(this_opt, "ypan"))
			ypan = 1;
		else if (!strcmp(this_opt, "ywrap"))
			ypan = 2;
		else if (!strncmp(this_opt, "tv=", 3)) {
			printk("detected \"tv=\"\n");
			printk("cmd line: %s\n", this_opt);
			if (!strncmp(this_opt + 3, "PAL", 3))
				tv_encoding = TV_ENC_PAL;
			else if (!strncmp(this_opt + 3, "NTSC", 4))
				tv_encoding = TV_ENC_NTSC;
		}
	}
	return 0;
}

int __init gcnfb_init(void)
{
	int video_cmap_len;
	int i;
	int err = 0;
	char *option = NULL;

	if (fb_get_options("gcnfb", &option)) {
		if (fb_get_options("gamecubefb", &option))
			return -ENODEV;
	}
	gcnfb_setup(option);

	/* detect current video mode */
	if (tv_encoding == TV_ENC_DETECT) {
		tv_encoding = ((gcn_video[0] >> 8) & 3) + 1;
	}

	gcnfb_defined.bits_per_pixel = 16;
	gcnfb_defined.xres = 640;
	gcnfb_defined.yres = (tv_encoding == TV_ENC_NTSC) ? 480 : 576;

	gcnfb_fix.line_length =
	    gcnfb_defined.xres * (gcnfb_defined.bits_per_pixel / 8);
	gcnfb_fix.smem_len = gcnfb_fix.line_length * gcnfb_defined.yres;
	gcnfb_fix.smem_start = GCN_XFB_START;

	gcnfb_fix.visual = (gcnfb_defined.bits_per_pixel == 8) ?
	    FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;

	if (!request_mem_region
	    (gcnfb_fix.smem_start, gcnfb_fix.smem_len, "gcnfb")) {
		printk(KERN_WARNING
		       "gcnfb: abort, cannot reserve video memory at %p\n",
		       (void *)gcnfb_fix.smem_start);
		/* We cannot make this fatal. Sometimes this comes from magic
		   spaces our resource handlers simply don't know about */
	}

	gcnfb_info.screen_base = ioremap(gcnfb_fix.smem_start, gcnfb_fix.smem_len);
	if (!gcnfb_info.screen_base) {
		printk(KERN_ERR
		       "gcnfb: abort, cannot ioremap video memory"
		       " at %p (%dk)\n",
		       (void *)gcnfb_fix.smem_start, gcnfb_fix.smem_len / 1024);
		err = -EIO;
		goto err_ioremap;
	}

	printk(KERN_INFO
	       "gcnfb: framebuffer at 0x%p, mapped to 0x%p, size %dk\n",
	       (void *)gcnfb_fix.smem_start, gcnfb_info.screen_base,
	       gcnfb_fix.smem_len / 1024);
	printk(KERN_INFO
	       "gcnfb: mode is %dx%dx%d, linelength=%d, pages=%d\n",
	       gcnfb_defined.xres, gcnfb_defined.yres,
	       gcnfb_defined.bits_per_pixel, gcnfb_fix.line_length,
	       0 /*screen_info.pages */ );

	gcnfb_defined.xres_virtual = gcnfb_defined.xres;
	gcnfb_defined.yres_virtual = gcnfb_defined.yres;
	ypan = 0;

	/* FIXME! Please, use here *real* values */
	/* some dummy values for timing to make fbset happy */
	gcnfb_defined.pixclock =
	    10000000 / gcnfb_defined.xres * 1000 / gcnfb_defined.yres;
	gcnfb_defined.left_margin = (gcnfb_defined.xres / 8) & 0xf8;
	gcnfb_defined.hsync_len = (gcnfb_defined.xres / 8) & 0xf8;

	if (gcnfb_defined.bits_per_pixel == 15) {
		gcnfb_defined.red.offset = 11;
		gcnfb_defined.red.length = 5;
		gcnfb_defined.green.offset = 6;
		gcnfb_defined.green.length = 5;
		gcnfb_defined.blue.offset = 1;
		gcnfb_defined.blue.length = 5;
		gcnfb_defined.transp.offset = 15;
		gcnfb_defined.transp.length = 1;
		video_cmap_len = 16;
	} else if (gcnfb_defined.bits_per_pixel == 16) {
		gcnfb_defined.red.offset = 11;
		gcnfb_defined.red.length = 5;
		gcnfb_defined.green.offset = 5;
		gcnfb_defined.green.length = 6;
		gcnfb_defined.blue.offset = 0;
		gcnfb_defined.blue.length = 5;
		gcnfb_defined.transp.offset = 0;
		gcnfb_defined.transp.length = 0;
		video_cmap_len = 16;
	} else {
		gcnfb_defined.red.length = 6;
		gcnfb_defined.green.length = 6;
		gcnfb_defined.blue.length = 6;
		video_cmap_len = 256;
	}

	gcnfb_fix.ypanstep = ypan ? 1 : 0;
	gcnfb_fix.ywrapstep = (ypan > 1) ? 1 : 0;

	gcnfb_info.fbops = &gcnfb_ops;
	gcnfb_info.var = gcnfb_defined;
	gcnfb_info.fix = gcnfb_fix;
	gcnfb_info.pseudo_palette = pseudo_palette;
	gcnfb_info.flags = FBINFO_FLAG_DEFAULT;

	if (fb_alloc_cmap(&gcnfb_info.cmap, video_cmap_len, 0)) {
		err = -ENOMEM;
		goto err_alloc_cmap;
	}

	if (register_framebuffer(&gcnfb_info) < 0) {
		err = -EINVAL;
		goto err_register_framebuffer;
	}

	unsigned int *VIDEO_Mode;

	if (tv_encoding == TV_ENC_NTSC)
		VIDEO_Mode = (unsigned int *)VIDEO_Mode640X480NtscYUV16;
	else
		VIDEO_Mode = (unsigned int *)VIDEO_Mode640X480Pal50YUV16;

	/* initialize video registers */
	for (i = 0; i < 7; i++) {
		gcn_video[i] = VIDEO_Mode[i];
	}
	gcn_video[8] = VIDEO_Mode[8];
	for (i = 10; i < 32; i++) {
		gcn_video[i] = VIDEO_Mode[i];
	}

	gcn_video[7] = 0x10000000 | (gcnfb_fix.smem_start >> 5);

/*
 * setting both fields to same source means half the resolution, but
 * reduces flickering a lot ...mmmh maybe worth a try as a last resort :/
 * gcn_video[9] = 0x10000000 | (gcnfb_fix.smem_start>>5);
 *
 */

	gcn_video[9] =
	    0x10000000 | ((gcnfb_fix.smem_start + gcnfb_fix.line_length) >> 5);

	printk(KERN_INFO "fb%d: %s frame buffer device\n",
	       gcnfb_info.node, gcnfb_info.fix.id);
	goto out;

err_register_framebuffer:
	fb_dealloc_cmap(&gcnfb_info.cmap);
err_alloc_cmap:
	iounmap(gcnfb_info.screen_base);
err_ioremap:
	release_mem_region(gcnfb_fix.smem_start, gcnfb_fix.smem_len);
out:
	return err;
}

static void __exit gcnfb_exit(void)
{
	unregister_framebuffer(&gcnfb_info);
	fb_dealloc_cmap(&gcnfb_info.cmap);
	iounmap(gcnfb_info.screen_base);
	release_mem_region(gcnfb_fix.smem_start, gcnfb_fix.smem_len);
}

module_init(gcnfb_init);
module_exit(gcnfb_exit);

