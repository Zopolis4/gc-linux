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

/* --------------------------------------------------------------------- */

static struct fb_var_screeninfo gamecubefb_defined __initdata = {
	.activate	= FB_ACTIVATE_NOW,
	.height		= -1,
	.width		= -1,
	.right_margin	= 32,
	.upper_margin	= 16,
	.lower_margin	= 4,
	.vsync_len	= 4,
	.vmode		= FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo gamecubefb_fix __initdata = {
	.id	= "GameCube",
	.type	= FB_TYPE_PACKED_PIXELS,
	.accel	= FB_ACCEL_NONE,
};

static struct fb_info fb_info;
static u32 pseudo_palette[17];

static int             ypan       = 0;  /* 0..nothing, 1..ypan, 2..ywrap */
static unsigned short  *pmi_base  = 0;
static void            (*pmi_start)(void);
static void            (*pmi_pal)(void);


/* --------------------------------------------------------------------- */
// This is Costis' RGB to YCbYCr conversion code. This looks like a bad
// hack, because we make the original cfbimgblit.c encode RGB 5:5:5 pixel
// data and convert it into YCbYCr on every write of an int into the
// framebuffer. But this is also quite good implementation, because YCbYCr
// means that two pixels are always encoded together and, while each one
// has its own luminance, they share the chrominance, so a putpixel() is
// not possible, and just hooking into settwopixels() solves all problems.
//
// What doesn't work correctly right now, is setting a single pixel, because
// the cfbimgblit.c code reads two pixels, assumes that it is RGB, changes
// one pixel, and we'll convert it into YCbYCr. This breaks the encoding.
// So we need to implement a 32 bit read from the framebuffer as well and
// return RGB encoded data.
#define CLAMP(x,l,h) ((x > h) ? h : ((x < l) ? l : x))

// 16:16 fixed point... hopefully not much accuracy is lost!
#define Ya 16843 // 0.257
#define Yb 33030 // 0.504
#define Yc 6423 // 0.098
#define Yd 1048576 // 16.0
#define Ye 32768 // 0.5

#define Cba -9699 // 0.148
#define Cbb -19071 // 0.291
#define Cbc 28770 // 0.439
#define Cbd 8388608 // 128.0
#define Cbe 32768 // 0.5

#define Cra 28770 // 0.439
#define Crb -24117 // 0.368
#define Crc -4653 // 0.071
#define Crd 8388608 // 128.0
#define Cre 32768 // 0.5

unsigned long GC_Video_RGBToYCbCrFixed (unsigned char r, unsigned char g, unsigned char b)
{
	unsigned long Y, Cb, Cr;

	Y = ((Ya * r) + (Yb * g) + (Yc * b) + Yd + Ye) >> 16;
	Cb = ((Cba * r) + (Cbb * g) + (Cbc * b) + Cbd + Cre) >> 16;
	Cr = ((Cra * r) + (Crb * g) + (Crc * b) + Crd + Cre) >> 16;

    Y  = CLAMP(Y , 16, 235);
    Cb = CLAMP(Cb, 16, 240);
    Cr = CLAMP(Cr, 16, 240);

	return (unsigned long)(((unsigned char)Y << 24) | ((unsigned char)Cb << 16) | ((unsigned char)Y << 8) | (unsigned char)Cr);
}

void gamecubefb_writel(unsigned long color, int *address)
{
        unsigned char r, g, b;
        unsigned long pa, pb;

        r = ((color >> 27) & 31) << 3;
        g = ((color >> 22) & 31) << 3;
        b = ((color >> 17) & 31) << 3;

        pa = GC_Video_RGBToYCbCrFixed (r, g, b);

        r = ((color >> 11) & 31) << 3;
        g = ((color >> 6) & 31) << 3;
        b = ((color >> 1) & 31) << 3;

        pb = GC_Video_RGBToYCbCrFixed (r, g, b);

        fb_writel_real((pa & 0xFF000000) | (pb & 0x0000FF00) |
                (((pa & 0x00FF0000) + (pb & 0x00FF0000)) >> 1) |
                (((pa & 0x000000FF) + (pb & 0x000000FF)) >> 1), address);
}

/* --------------------------------------------------------------------- */

static int gamecubefb_pan_display(struct fb_var_screeninfo *var,
                              struct fb_info *info)
{
	return 0;
}

static void vesa_setpalette(int regno, unsigned red, unsigned green, unsigned blue)
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
		vesa_setpalette(regno,red,green,blue);
		break;
	case 16:
		if (info->var.red.offset == 10) {
			/* 1:5:5:5 */
			((u32*) (info->pseudo_palette))[regno] =
					((red   & 0xf800) >>  1) |
					((green & 0xf800) >>  6) |
					((blue  & 0xf800) >> 11);
		} else {
			/* 0:5:6:5 */
			((u32*) (info->pseudo_palette))[regno] =
					((red   & 0xf800)      ) |
					((green & 0xfc00) >>  5) |
					((blue  & 0xf800) >> 11);
		}
		break;
	case 24:
		red   >>= 8;
		green >>= 8;
		blue  >>= 8;
		((u32 *)(info->pseudo_palette))[regno] =
			(red   << info->var.red.offset)   |
			(green << info->var.green.offset) |
			(blue  << info->var.blue.offset);
		break;
	case 32:
		red   >>= 8;
		green >>= 8;
		blue  >>= 8;
		((u32 *)(info->pseudo_palette))[regno] =
			(red   << info->var.red.offset)   |
			(green << info->var.green.offset) |
			(blue  << info->var.blue.offset);
		break;
    }
    return 0;
}

void gc_blit() {
	printk("BLIT\n\n\n");
}

static struct fb_ops gamecubefb_ops = {
	.owner		= THIS_MODULE,
#if 0
	.fb_setcolreg	= gc_blit,
	.fb_pan_display	= gc_blit,
	.fb_fillrect	= gc_blit,
	.fb_copyarea	= gc_blit,
	.fb_imageblit	= gc_blit,
	.fb_cursor	= gc_blit,
#endif
#if 1
	.fb_setcolreg	= gamecubefb_setcolreg,
	.fb_pan_display	= gamecubefb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_cursor	= soft_cursor,
#endif
};

int __init gamecubefb_setup(char *options)
{
	char *this_opt;

	if (!options || !*options)
		return 0;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt) continue;

		if (! strcmp(this_opt, "redraw"))
			ypan=0;
		else if (! strcmp(this_opt, "ypan"))
			ypan=1;
		else if (! strcmp(this_opt, "ywrap"))
			ypan=2;
	}
	return 0;
}

int __init gamecubefb_init(void)
{
	int video_cmap_len;
	int i;

	gamecubefb_defined.bits_per_pixel = 16;
	gamecubefb_defined.xres = 640;
	gamecubefb_defined.yres = 576;
	gamecubefb_fix.line_length = 640*2;
	gamecubefb_fix.smem_len = 640*576*2;
	gamecubefb_fix.smem_start = (24*1024*1024)-gamecubefb_fix.smem_len;
	gamecubefb_fix.visual   = (gamecubefb_defined.bits_per_pixel == 8) ?
		FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;

	if (!request_mem_region(gamecubefb_fix.smem_start, gamecubefb_fix.smem_len, "gamecubefb")) {
		printk(KERN_WARNING
		       "gamecubefb: abort, cannot reserve video memory at 0x%lx\n",
			gamecubefb_fix.smem_start);
		/* We cannot make this fatal. Sometimes this comes from magic
		   spaces our resource handlers simply don't know about */
	}

        fb_info.screen_base = ioremap(gamecubefb_fix.smem_start, gamecubefb_fix.smem_len);
	if (!fb_info.screen_base) {
		release_mem_region(gamecubefb_fix.smem_start, gamecubefb_fix.smem_len);
		printk(KERN_ERR
		       "gamecubefb: abort, cannot ioremap video memory 0x%x @ 0x%lx\n",
			gamecubefb_fix.smem_len, gamecubefb_fix.smem_start);
		return -EIO;
	}

	//MISTFIX
	//fb_info.screen_base = gamecubefb_fix.smem_start;

	printk(KERN_INFO "gamecubefb: framebuffer at 0x%lx, mapped to 0x%p, size %dk\n",
	       gamecubefb_fix.smem_start, fb_info.screen_base, gamecubefb_fix.smem_len/1024);
	printk(KERN_INFO "gamecubefb: mode is %dx%dx%d, linelength=%d, pages=%d\n",
	       gamecubefb_defined.xres, gamecubefb_defined.yres, gamecubefb_defined.bits_per_pixel, gamecubefb_fix.line_length, 0); //screen_info.pages);

	gamecubefb_defined.xres_virtual = 640;
	gamecubefb_defined.yres_virtual = 576;
	ypan = 0;

	/* some dummy values for timing to make fbset happy */
	gamecubefb_defined.pixclock     = 10000000 / gamecubefb_defined.xres * 1000 / gamecubefb_defined.yres;
	gamecubefb_defined.left_margin  = (gamecubefb_defined.xres / 8) & 0xf8;
	gamecubefb_defined.hsync_len    = (gamecubefb_defined.xres / 8) & 0xf8;

	if (gamecubefb_defined.bits_per_pixel > 8) {
		gamecubefb_defined.red.offset    = 11;
		gamecubefb_defined.red.length    = 5;
		gamecubefb_defined.green.offset  = 6;
		gamecubefb_defined.green.length  = 5;
		gamecubefb_defined.blue.offset   = 1;
		gamecubefb_defined.blue.length   = 5;
		gamecubefb_defined.transp.offset = 15;
		gamecubefb_defined.transp.length = 1;
		video_cmap_len = 16;
	} else {
		gamecubefb_defined.red.length   = 6;
		gamecubefb_defined.green.length = 6;
		gamecubefb_defined.blue.length  = 6;
		video_cmap_len = 256;
	}

	gamecubefb_fix.ypanstep  = ypan     ? 1 : 0;
	gamecubefb_fix.ywrapstep = (ypan>1) ? 1 : 0;

	fb_info.fbops = &gamecubefb_ops;
	fb_info.var = gamecubefb_defined;
	fb_info.fix = gamecubefb_fix;
	fb_info.pseudo_palette = pseudo_palette;
	fb_info.flags = FBINFO_FLAG_DEFAULT;

	fb_alloc_cmap(&fb_info.cmap, video_cmap_len, 0);

	if (register_framebuffer(&fb_info)<0)
		return -EINVAL;

volatile static unsigned int *gamecube_video = (unsigned int*) 0xCC002000;
static const unsigned int VIDEO_Mode640X480Pal50YUV16[32] = {
0x11F50101, 0x4B6A01B0, 0x02F85640, 0x00010023,
0x00000024, 0x4D2B4D6D, 0x4D8A4D4C, 0x00435A4E,
0x00000000, 0x00435A4E, 0x00000000, 0x013C0144,
0x113901B1, 0x10010001, 0x00010001, 0x00010001,
0x00000000, 0x00000000, 0x28500100, 0x1AE771F0,
0x0DB4A574, 0x00C1188E, 0xC4C0CBE2, 0xFCECDECF,
0x13130F08, 0x00080C0F, 0x00FF0000, 0x00000000,
0x02800000, 0x000000FF, 0x00FF00FF, 0x00FF00FF};

	// initialize screen
	for(i=0; i<32; i++) {
		gamecube_video[i] = VIDEO_Mode640X480Pal50YUV16[i];
	}
	gamecube_video[7] = 0x10000000 | (gamecubefb_fix.smem_start>>5);
	printk("POKE(%x,%x)\n", &gamecube_video[7], 0x10000000 | (gamecubefb_fix.smem_start>>5));
	gamecube_video[9] = 0x10000000 | ((gamecubefb_fix.smem_start+gamecubefb_fix.line_length)>>5);
	printk("POKE(%x,%x)\n", &gamecube_video[9], (gamecubefb_fix.smem_start+gamecubefb_fix.line_length)>>5);

#if 1
	/* clear screen */
	int c = 640*576/2;
	//unsigned long *p = (unsigned long*)fb_info.screen_base;
	unsigned long *p = (unsigned long*)fb_info.screen_base;
	while (c--)
		*p++ = 0x00800080;
#endif
	printk("Clearing screen at %x\n",fb_info.screen_base);

	printk(KERN_INFO "fb%d: %s frame buffer device\n",
	       fb_info.node, fb_info.fix.id);

	return 0;
}

/*
 * Overrides for Emacs so that we follow Linus's tabbing style.
 * ---------------------------------------------------------------------------
 * Local variables:
 * c-basic-offset: 8
 * End:
 */

MODULE_LICENSE("GPL");
