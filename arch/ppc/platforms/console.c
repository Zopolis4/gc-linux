#include "console.h"
#include <linux/string.h>
#include <linux/console.h>
#include <linux/font.h>

#define COLOR_WHITE 0xFF80FF80
#define COLOR_BLACK 0x00800080

static struct console_data_s *default_console;
void console_do_init(void);

void con_puts(const char *s)
{
	if (!default_console)
		console_do_init();
	console_puts(default_console, s);
}

void con_putc(char c)
{
	if (!default_console)
		console_do_init();
		console_putc(default_console, c);
}

int set_color(int background, int foreground)
{
	default_console->foreground = foreground;
	default_console->background = background;
	return 0;
}

struct console_data_s console;
//extern unsigned char console_font_8x8[];
extern struct font_desc font_vga_8x16;

void xconsole_init(struct console_data_s *con, void *framebuffer, int xres, int yres, int stride)
{
	con->framebuffer = framebuffer;
	con->xres = xres;
	con->yres = yres;
	con->border_left = 0;
	con->border_top  = 0;
	con->border_right = con->xres;
	con->border_bottom = con->yres;
	con->stride = stride;
	con->cursor_x = con->cursor_y = 0;
	
	con->font = font_vga_8x16.data;
	
	con->foreground = COLOR_WHITE;
	con->background = COLOR_BLACK;
	
	con->scrolled_lines = 0;
	int c = con->xres * con->yres / 2;
	unsigned long *p = (unsigned long*)con->framebuffer;
	while (c--)
		*p++ = con->background;
	default_console = con;
}

#define FONT_XSIZE  8
#define FONT_YSIZE  16
#define FONT_XFACTOR 1
#define FONT_YFACTOR 1
#define FONT_XGAP   2
#define FONT_YGAP   0

void console_drawc(struct console_data_s *con, int x, int y, unsigned char c)
{
	x >>= 1;
	int ax, ay;
	unsigned long *ptr = (unsigned long*)(con->framebuffer + con->stride * y + x * 4);
	for (ay = 0; ay < FONT_YSIZE; ay++)
	{
#if FONT_XFACTOR == 2
		for (ax = 0; ax < 8; ax++)
		{
			unsigned long color;
			if ((con->font[c * FONT_YSIZE + ay] << ax) & 0x80)
				color = con->foreground;
			else
				color = con->background;
#if FONT_YFACTOR == 2
				// pixel doubling: we write u32
			ptr[ay * 2 * con->stride/4 + ax] = color;
				// line doubling
			ptr[(ay * 2 +1) * con->stride/4 + ax] = color;
#else
			ptr[ay * con->stride/4 + ax] = color;
#endif
		}
#else
		for (ax = 0; ax < 4; ax ++)
		{
			unsigned long color[2];
			int bits = (con->font[c * FONT_YSIZE + ay] << (ax*2));
			if (bits & 0x80)
				color[0] = con->foreground;
			else
				color[0] = con->background;
			if (bits & 0x40)
				color[1] = con->foreground;
			else
				color[1] = con->background;
			ptr[ay * con->stride/4 + ax] = (color[0] & 0xFFFF00FF) | (color[1] & 0x0000FF00);
		}
#endif
	}
}

void console_puts(struct console_data_s *con, const char *string)
{
	while (*string)
		console_putc(con, *string++);
}

void console_putc(struct console_data_s *con, char c)
{
	switch (c)
	{
	case '\n':
		con->cursor_y += FONT_YSIZE * FONT_YFACTOR + FONT_YGAP;
		con->cursor_x = con->border_left;
		break;
	default:
		console_drawc(con, con->cursor_x, con->cursor_y, c);
		con->cursor_x += FONT_XSIZE * FONT_XFACTOR + FONT_XGAP;
		if ((con->cursor_x + (FONT_XSIZE * FONT_XFACTOR)) > con->border_right)
		{
			con->cursor_y += FONT_YSIZE * FONT_YFACTOR + FONT_YGAP;
			con->cursor_x = con->border_left;
		}
	}
	if ((con->cursor_y + FONT_YSIZE * FONT_YFACTOR) >= con->border_bottom)
	{
		memcpy(con->framebuffer, 
				con->framebuffer + con->stride * (FONT_YSIZE * FONT_YFACTOR + FONT_YGAP), 
				con->stride * con->yres - FONT_YSIZE);
		int cnt = (con->stride * (FONT_YSIZE * FONT_YFACTOR + FONT_YGAP)) / 4;
		unsigned long *ptr = (unsigned long*)(con->framebuffer + con->stride * (con->yres - FONT_YSIZE));
		while (cnt--)
			 *ptr++ = con->background;
		con->cursor_y -= FONT_YSIZE * FONT_YFACTOR + FONT_YGAP;
	}
	
	flush_dcache_range(con->framebuffer, con->framebuffer + con->stride * con->yres);
}

static struct console gamecube_console = {
	.name =		"debug",
	.flags =	CON_PRINTBUFFER,
	.index =	-1,
};

static void gamecube_console_write(struct console *co, const char *b,
				    unsigned int count)
{
	while (count--)
		console_putc(default_console, *b++);
}

static struct console_data_s con;

void console_do_init(void)
{
	xconsole_init(&con, 0xd0c00000, 640, 576, 640*2);
	con_puts("console intiialized.\n");
	gamecube_console.write = gamecube_console_write;
	register_console(&gamecube_console);
}
