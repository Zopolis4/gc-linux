/*
 * framebuffer driver for the GameCube's "Flipper" chipset
 *
 * (c) 2004 Michael Steil <mist@c64.org>
 * based on vesafb:
 * (c) 1998 Gerd Knorr <kraxel@goldbach.in-berlin.de>
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

volatile static unsigned int *gamecube_video = (unsigned int *)0xCC002000;

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

static struct fb_var_screeninfo gcfb_defined __initdata = {
	.activate = FB_ACTIVATE_NOW,
	.height = -1,
	.width = -1,
	.right_margin = 32,
	.upper_margin = 16,
	.lower_margin = 4,
	.vsync_len = 4,
	.vmode = FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo gcfb_fix __initdata = {
	.id = "GameCube",
	.type = FB_TYPE_PACKED_PIXELS,
	.accel = FB_ACCEL_NONE,
};

#define TV_ENC_DETECT 0
#define TV_ENC_NTSC 1
#define TV_ENC_PAL 2
static int tv_encoding __initdata = TV_ENC_DETECT;

static struct fb_info gcfb_info;
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

void gamecubefb_writel(register unsigned long rgbrgb, register int *address)
{
	uint16_t *rgb = (uint16_t *) & rgbrgb;
	fb_writel_real(rgbrgb16toycbycr(rgb[0], rgb[1]), address);
}

static int gamecubefb_pan_display(struct fb_var_screeninfo *var,
				  struct fb_info *info)
{
	return 0;
}

static void vesa_setpalette(int regno, unsigned red, unsigned green,
			    unsigned blue)
{
}

static int gamecubefb_setcolreg(unsigned regno, unsigned red, unsigned green,
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

static struct fb_ops gamecubefb_ops = {
	.owner = THIS_MODULE,
	.fb_setcolreg = gamecubefb_setcolreg,
	.fb_pan_display = gamecubefb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_cursor = soft_cursor,
};

int __init gamecubefb_setup(char *options)
{
	char *this_opt;

	printk("options = %s\n", options);
	if (!options || !*options)
		return 0;

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

static int __init gamecubefb_init(void)
{
	int video_cmap_len;
	int i;
	int err = 0;
	char *option = NULL;

	if (fb_get_options("gamecubefb", &option))
		return -ENODEV;
	gamecubefb_setup(option);

	// detect current video mode
	if (tv_encoding == TV_ENC_DETECT) {
		tv_encoding = ((gamecube_video[0] >> 8) & 3) + 1;
	}

	gcfb_defined.bits_per_pixel = 16;
	gcfb_defined.xres = 640;
	gcfb_defined.yres = (tv_encoding == TV_ENC_NTSC) ? 480 : 576;

	gcfb_fix.line_length =
	    gcfb_defined.xres * (gcfb_defined.bits_per_pixel / 8);
	gcfb_fix.smem_len = gcfb_fix.line_length * gcfb_defined.yres;
	/* place XFB at end of RAM */
	gcfb_fix.smem_start = (24 * 1024 * 1024) - gcfb_fix.smem_len;

	gcfb_fix.visual = (gcfb_defined.bits_per_pixel == 8) ?
	    FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;

	if (!request_mem_region
	    (gcfb_fix.smem_start, gcfb_fix.smem_len, "gamecubefb")) {
		printk(KERN_WARNING
		       "gamecubefb: abort, cannot reserve video memory at %p\n",
		       (void *)gcfb_fix.smem_start);
		/* We cannot make this fatal. Sometimes this comes from magic
		   spaces our resource handlers simply don't know about */
	}

	gcfb_info.screen_base = ioremap(gcfb_fix.smem_start, gcfb_fix.smem_len);
	if (!gcfb_info.screen_base) {
		printk(KERN_ERR
		       "gamecubefb: abort, cannot ioremap video memory"
		       " at %p (%dk)\n",
		       (void *)gcfb_fix.smem_start, gcfb_fix.smem_len / 1024);
		err = -EIO;
		goto err_ioremap;
	}

	printk(KERN_INFO
	       "gamecubefb: framebuffer at 0x%p, mapped to 0x%p, size %dk\n",
	       (void *)gcfb_fix.smem_start, gcfb_info.screen_base,
	       gcfb_fix.smem_len / 1024);
	printk(KERN_INFO
	       "gamecubefb: mode is %dx%dx%d, linelength=%d, pages=%d\n",
	       gcfb_defined.xres, gcfb_defined.yres,
	       gcfb_defined.bits_per_pixel, gcfb_fix.line_length,
	       0 /*screen_info.pages */ );

	gcfb_defined.xres_virtual = gcfb_defined.xres;
	gcfb_defined.yres_virtual = gcfb_defined.yres;
	ypan = 0;

	/* FIXME! Please, use here *real* values */
	/* some dummy values for timing to make fbset happy */
	gcfb_defined.pixclock =
	    10000000 / gcfb_defined.xres * 1000 / gcfb_defined.yres;
	gcfb_defined.left_margin = (gcfb_defined.xres / 8) & 0xf8;
	gcfb_defined.hsync_len = (gcfb_defined.xres / 8) & 0xf8;

	if (gcfb_defined.bits_per_pixel == 15) {
		gcfb_defined.red.offset = 11;
		gcfb_defined.red.length = 5;
		gcfb_defined.green.offset = 6;
		gcfb_defined.green.length = 5;
		gcfb_defined.blue.offset = 1;
		gcfb_defined.blue.length = 5;
		gcfb_defined.transp.offset = 15;
		gcfb_defined.transp.length = 1;
		video_cmap_len = 16;
	} else if (gcfb_defined.bits_per_pixel == 16) {
		gcfb_defined.red.offset = 11;
		gcfb_defined.red.length = 5;
		gcfb_defined.green.offset = 5;
		gcfb_defined.green.length = 6;
		gcfb_defined.blue.offset = 0;
		gcfb_defined.blue.length = 5;
		gcfb_defined.transp.offset = 0;
		gcfb_defined.transp.length = 0;
		video_cmap_len = 16;
	} else {
		gcfb_defined.red.length = 6;
		gcfb_defined.green.length = 6;
		gcfb_defined.blue.length = 6;
		video_cmap_len = 256;
	}

	gcfb_fix.ypanstep = ypan ? 1 : 0;
	gcfb_fix.ywrapstep = (ypan > 1) ? 1 : 0;

	gcfb_info.fbops = &gamecubefb_ops;
	gcfb_info.var = gcfb_defined;
	gcfb_info.fix = gcfb_fix;
	gcfb_info.pseudo_palette = pseudo_palette;
	gcfb_info.flags = FBINFO_FLAG_DEFAULT;

	if (fb_alloc_cmap(&gcfb_info.cmap, video_cmap_len, 0)) {
		err = -ENOMEM;
		goto err_alloc_cmap;
	}

	if (register_framebuffer(&gcfb_info) < 0) {
		err = -EINVAL;
		goto err_register_framebuffer;
	}

	/* fill framebuffer memory with black color */
	int c = gcfb_defined.xres * gcfb_defined.yres / 2;
	volatile unsigned long *p = (unsigned long *)gcfb_info.screen_base;
	while (c--)
		writel(0x00800080, p++);

	unsigned int *VIDEO_Mode;

	if (tv_encoding == TV_ENC_NTSC)
		VIDEO_Mode = (unsigned int *)VIDEO_Mode640X480NtscYUV16;
	else
		VIDEO_Mode = (unsigned int *)VIDEO_Mode640X480Pal50YUV16;

	/* initialize video registers */
	for (i = 0; i < 7; i++) {
		gamecube_video[i] = VIDEO_Mode[i];
	}
	gamecube_video[8] = VIDEO_Mode[8];
	for (i = 10; i < 32; i++) {
		gamecube_video[i] = VIDEO_Mode[i];
	}

	gamecube_video[7] = 0x10000000 | (gcfb_fix.smem_start >> 5);

// setting both fields to same source means half the resolution, but
// reduces flickering a lot ...mmmh maybe worth a try as a last resort :/
// gamecube_video[9] = 0x10000000 | (gcfb_fix.smem_start>>5);

	gamecube_video[9] =
	    0x10000000 | ((gcfb_fix.smem_start + gcfb_fix.line_length) >> 5);

	printk(KERN_INFO "fb%d: %s frame buffer device\n",
	       gcfb_info.node, gcfb_info.fix.id);
	goto out;

err_register_framebuffer:
	fb_dealloc_cmap(&gcfb_info.cmap);
err_alloc_cmap:
	iounmap(gcfb_info.screen_base);
err_ioremap:
	release_mem_region(gcfb_fix.smem_start, gcfb_fix.smem_len);
out:
	return err;
}

static void __exit gamecubefb_exit(void)
{
	release_mem_region(gcfb_fix.smem_start, gcfb_fix.smem_len);
	iounmap(gcfb_info.screen_base);
	fb_dealloc_cmap(&gcfb_info.cmap);
	unregister_framebuffer(&gcfb_info);
}

/*
 * Overrides for Emacs so that we follow Linus's tabbing style.
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-basic-offset: 8
 * End:
 */

module_init(gamecubefb_init);
module_exit(gamecubefb_exit);
MODULE_LICENSE("GPL");
