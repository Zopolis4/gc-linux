#include <linux/init.h>
#include <linux/time.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////
// must be MOVED LATER
///////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXI_READ  0
#define EXI_WRITE 1

/* exi_select: enable chip select, set speed */

static int selected = 0;
static void exi_select(int channel, int device, int freq)
{
	volatile unsigned long *exi =
	    (volatile unsigned long *) 0xCC006800;
	selected++;
	if (selected != 1)
		panic("-------- select while selected!\n");
	long d;
	// exi_select
	d = exi[channel * 5];
	d &= 0x405;
	d |= ((1 << device) << 7) | (freq << 4);
	exi[channel * 5] = d;
}

/* disable chipselect */
static void exi_deselect(int channel)
{
	volatile unsigned long *exi =
	    (volatile unsigned long *) 0xCC006800;
	selected--;
	if (selected)
		panic("deselect broken!");
	exi[channel * 5] &= 0x405;
}

/* dirty way for asynchronous reads */
static void *exi_last_addr;
static int exi_last_len;

/* mode?Read:Write len bytes to/from channel */
/* when read, data will be written back in exi_sync */
static void exi_imm(int channel, void *data, int len, int mode, int zero)
{
	volatile unsigned long *exi =
	    (volatile unsigned long *) 0xCC006800;
	if (mode == EXI_WRITE)
		exi[channel * 5 + 4] = *(unsigned long *) data;
	exi[channel * 5 + 3] = ((len - 1) << 4) | (mode << 2) | 1;
	if (mode == EXI_READ) {
		exi_last_addr = data;
		exi_last_len = len;
	} else {
		exi_last_addr = 0;
		exi_last_len = 0;
	}
}

/* Wait until transfer is done, write back data */
static void exi_sync(int channel)
{
	volatile unsigned long *exi =
	    (volatile unsigned long *) 0xCC006800;
	while (exi[channel * 5 + 3] & 1);

	if (exi_last_addr) {
		int i;
		unsigned long d;
		d = exi[channel * 5 + 4];
		for (i = 0; i < exi_last_len; ++i)
			((unsigned char *) exi_last_addr)[i] =
			    (d >> ((3 - i) * 8)) & 0xFF;
	}
}

/* simple wrapper for transfers > 4bytes */
static void exi_imm_ex(int channel, void *data, int len, int mode)
{
	unsigned char *d = (unsigned char *) data;
	while (len) {
		int tc = len;
		if (tc > 4)
			tc = 4;
		exi_imm(channel, d, tc, mode, 0);
		exi_sync(channel);
		len -= tc;
		d += tc;
	}
}

///////////////////////////////////////////////////////////7

#define RTC_OFFSET 946684800L

static int bias = 0;

static void read_sram(unsigned char *abuf)
{
	unsigned long a;

	// Select the SRAM device.
	exi_select(0, 1, 3);

	// Send the appropriate command.
	a = 0x20000100;
	exi_imm(0, &a, 4, 1, 0);
	exi_sync(0);

	// Read the SRAM data!
	exi_imm_ex(0, abuf, 64, 0);

	// Deselect the SRAM device.
	exi_deselect(0);

	return;
}

long __init gamecube_time_init(void)
{
	char sram[64];
	int *pbias = (int *)&sram[0xC];
	read_sram(sram);
	bias = *pbias;
	return 0;
}

static unsigned long get_rtc(void)
{
	unsigned long a = 0L;

	// Select the RTC device.
	exi_select(0, 1, 3);

	// Send the appropriate command.
	a = 0x20000000;
	exi_imm(0, &a, 4, 1, 0);
	exi_sync(0);
	// Read the time and date value!
	exi_imm(0, &a, 4, 0, 0);
	exi_sync(0);

	// Deselect the RTC device.
	exi_deselect(0);

	return a;
}

static void set_rtc(unsigned long aval)
{
	unsigned long a;

	// Select the RTC device.
	exi_select(0, 1, 3);

	// Send the appropriate command.
	a = 0xA0000000;
	exi_imm(0, &a, 4, 1, 0);
	exi_sync(0);

	// Set the new time and date value!
	exi_imm(0, &aval, 4, 1, 0);
	exi_sync(0);

	// Deselect the RTC device.
	exi_deselect(0);
}

unsigned long gamecube_get_rtc_time(void)
{
	return get_rtc() + bias + RTC_OFFSET;
}

int gamecube_set_rtc_time(unsigned long nowtime)
{
	set_rtc(nowtime - RTC_OFFSET - bias);

	return 1;
}

