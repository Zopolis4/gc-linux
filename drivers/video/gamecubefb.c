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
	.id	= "VESA VGA",
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

	gamecubefb_fix.smem_start = 0xd0c00000;
	gamecubefb_defined.bits_per_pixel = 16;
	gamecubefb_defined.xres = 640;
	gamecubefb_defined.yres = 576;
	gamecubefb_fix.line_length = 640*2;
	gamecubefb_fix.smem_len = 640*576*2;
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
	fb_info.screen_base = gamecubefb_fix.smem_start;

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
		gamecubefb_defined.red.offset    = 0;
		gamecubefb_defined.red.length    = 5;
		gamecubefb_defined.green.offset  = 5;
		gamecubefb_defined.green.length  = 5;
		gamecubefb_defined.blue.offset   = 10;
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
	printk("ops set\n");
	fb_info.var = gamecubefb_defined;
	fb_info.fix = gamecubefb_fix;
	fb_info.pseudo_palette = pseudo_palette;
	fb_info.flags = FBINFO_FLAG_DEFAULT;

	fb_alloc_cmap(&fb_info.cmap, video_cmap_len, 0);

	if (register_framebuffer(&fb_info)<0)
		return -EINVAL;

#if 0
	/* clear screen */
	int c = 640*576/2;
	//unsigned long *p = (unsigned long*)fb_info.screen_base;
	unsigned long *p = (unsigned long*)gamecubefb_fix.smem_start;
	while (c--)
		*p++ = 0x00800080;
#endif

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
