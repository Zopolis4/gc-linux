#ifndef __console_h
#define __console_h

struct console_data_s
{
	unsigned char *framebuffer;
	int xres, yres, stride;
	
	unsigned char *font;
	
	int cursor_x, cursor_y;
	int foreground, background;
	
	int border_left, border_right, border_top, border_bottom;
	
	int scrolled_lines;
};

int set_color(int background, int foreground);

void xconsole_init(struct console_data_s *con, void *framebuffer, int xres, int yres, int stride);
void console_puts(struct console_data_s *con, const char *string);
void console_putc(struct console_data_s *con, char c);

#endif
