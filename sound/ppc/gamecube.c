#include <sound/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <sound/core.h>
#include <sound/pcm.h>
#define SNDRV_GET_ID
#include <sound/initval.h>

#define DSP_IRQ 6
/* #define GAMECUBE_AUDIO_DEBUG */

#if 0
#include <asm/io.h>

#define AUDIO_DSP_CONTROL	(iobase + 0x0a)
#define AUDIO_IRQ_CAUSE		(iobase + 0x10)
#define AUDIO_DMA_STARTH	(iobase + 0x30)
#define AUDIO_DMA_STARTL	(iobase + 0x32)
#define AUDIO_DMA_LENGTH	(iobase + 0x36)
#define AUDIO_DMA_LEFT		(iobase + 0x3a)
#define AUDIO_INTERFACE_ADDR	0xCC005000
#endif

#define AUDIO_DSP_CONTROL   *(volatile u_int16_t *)(0xCC00500a)
#define AUDIO_IRQ_CAUSE     *(volatile u_int16_t *)(0xCC005010)

/* from CrowTRobo's audio demo */

#define AUDIO_DMA_STARTH    *(volatile u_int16_t *)(0xCC005030)
#define AUDIO_DMA_STARTL    *(volatile u_int16_t *)(0xCC005032)
#define AUDIO_DMA_LENGTH    *(volatile u_int16_t *)(0xCC005036)
#define AUDIO_DMA_LEFT      *(volatile u_int16_t *)(0xCC00503a)
#define AUDIO_STREAM_STATUS *(volatile u_int32_t *)(0xCC006C00)

#define LoadSample(addr, len) AUDIO_DMA_STARTH = addr >> 16;       \
                              AUDIO_DMA_STARTL = addr & 0xffff;    \
                              AUDIO_DMA_LENGTH = (AUDIO_DMA_LENGTH & 0x8000) | (len >> 5)
#define StartSample()  AUDIO_DMA_LENGTH |= 0x8000
#define StopSample()   AUDIO_DMA_LENGTH &= 0x7fff
#define SetFreq32KHz() AUDIO_STREAM_STATUS |= 0x40
#define SetFreq48KHz() AUDIO_STREAM_STATUS &= 0xffffffbf


MODULE_AUTHOR("me!");
MODULE_DESCRIPTION("GameCube sound stuff");
MODULE_LICENSE("GPL");

#define chip_t gamecube_t

static int index = SNDRV_DEFAULT_IDX1;		/* Index 0-MAX */
static char *id = SNDRV_DEFAULT_STR1;		/* ID for this card */

static unsigned long iobase;

typedef struct snd_gamecube {
	snd_card_t *card;
	snd_pcm_t *pcm;
	snd_pcm_substream_t *playback_substream;
	snd_pcm_substream_t *capture_substream;
	spinlock_t reg_lock;
	int dma_size;
	int period_size;
	int nperiods;
	volatile int cur_period;
	volatile int start_play;
	volatile int stop_play;
} gamecube_t;

static gamecube_t *gamecube_audio = NULL;

static snd_pcm_hardware_t snd_gamecube_playback = {
	.info =		(SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
			 SNDRV_PCM_INFO_BLOCK_TRANSFER |
			 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =	SNDRV_PCM_FMTBIT_S16_BE,
	.rates =	/* SNDRV_PCM_RATE_8000_48000, */
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000,
	.rate_min =	32000,
	.rate_max =	48000,
	.channels_min =		2,
	.channels_max =		2,
	.buffer_bytes_max =	32768,
	.period_bytes_min =	32,
	.period_bytes_max =	32768,
	.periods_min =		1,
	.periods_max =		1024,
};

static int snd_gamecube_open(snd_pcm_substream_t *substream)
{
	gamecube_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;

	printk(KERN_ALERT "pcm open\n");
	chip->playback_substream = substream;
	runtime->hw = snd_gamecube_playback;

	/* align to 32 bytes */ 
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);

	return 0;
}

static int snd_gamecube_close(snd_pcm_substream_t * substream)
{
	gamecube_t *chip = snd_pcm_substream_chip(substream);

	printk(KERN_ALERT "pcm close\n");
	chip->playback_substream = NULL;
	return 0;
}

static int snd_gamecube_hw_params(snd_pcm_substream_t * substream,
		snd_pcm_hw_params_t * hw_params)
{
	/* printk(KERN_ALERT "snd_gamecube_hw_params\n"); */
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

static int snd_gamecube_hw_free(snd_pcm_substream_t * substream)
{
	/* printk(KERN_ALERT "snd_gamecube_hw_free\n"); */
	return snd_pcm_lib_free_pages(substream);
}

static int snd_gamecube_prepare(snd_pcm_substream_t * substream)
{
	gamecube_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	
	/* printk(KERN_ALERT "snd_gamecube_prepare\n"); */
	printk("prepare: rate=%i, channels=%i, sample_bits=%i\n",
			runtime->rate, runtime->channels, runtime->sample_bits);
	printk("prepare: format=%i, access=%i\n",
			runtime->format, runtime->access);

	/* set requested samplerate */
	switch (runtime->rate) {
		case 32000:
			SetFreq32KHz();
			break;
		case 48000:
			SetFreq48KHz();
			break;
		default:
			printk("unsupported rate: %i!\n", runtime->rate);
			return -EINVAL;
	}

	return 0;
}

static int snd_gamecube_trigger(snd_pcm_substream_t * substream, int cmd)
{
	gamecube_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;

	/* printk(KERN_ALERT "snd_gamecube_trigger\n"); */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* do something to start the PCM engine */
		printk(KERN_ALERT "PCM_TRIGGER_START\n");
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			chip->dma_size = snd_pcm_lib_buffer_bytes(substream);
			chip->period_size = snd_pcm_lib_period_bytes(substream);
			chip->nperiods = chip->dma_size / chip->period_size;
			chip->cur_period = 0;
			chip->stop_play = 0;
			chip->start_play = 1;

			printk(KERN_ALERT "stream is PCM_PLAYBACK, dma_area=0x%p dma_size=%i\n",
				runtime->dma_area, chip->dma_size);
			printk(KERN_ALERT "%i periods of %i bytes\n",
				chip->nperiods, chip->period_size);
			
			flush_dcache_range(runtime->dma_area, runtime->dma_area + chip->period_size);
			LoadSample((u_int32_t) runtime->dma_area, chip->period_size);
			StartSample();
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		/* do something to stop the PCM engine */
		printk(KERN_ALERT "PCM_TRIGGER_STOP\n");

		chip->stop_play = 1;
		/* StopSample(); */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static snd_pcm_uframes_t snd_gamecube_pointer(snd_pcm_substream_t * substream)
{
	gamecube_t *chip = snd_pcm_substream_chip(substream);
	snd_pcm_runtime_t *runtime = substream->runtime;
	int left, bytes;

	/* printk(KERN_ALERT "snd_gamecube_pointer\n"); */
	left = AUDIO_DMA_LEFT << 5;
	bytes = chip->period_size * (chip->cur_period + 1);
	/* bytes = snd_pcm_lib_buffer_bytes(substream); */

#ifdef GAMECUBE_AUDIO_DEBUG
	printk("pointer: %i of %i(%i) bytes left, period #%i\n", left, chip->period_size ,bytes, chip->cur_period);
#endif
	return bytes_to_frames(runtime, bytes - left);
}

static irqreturn_t snd_gamecube_interrupt(int irq, void *dev, struct pt_regs *regs)
{
	gamecube_t *chip = (gamecube_t *) dev;
	unsigned long val = AUDIO_DSP_CONTROL;

	if (val & 0x100) {
		u_int32_t addr;
	
#ifdef GAMECUBE_AUDIO_DEBUG
		printk("DSP interrupt! period #%i\n", chip->cur_period);
#endif
		if (chip->start_play) {
			chip->start_play = 0;
		} else if (chip->stop_play) {
			StopSample();
		} else {
			StopSample();
			
			if (chip->cur_period < (chip->nperiods - 1)) {
				chip->cur_period++;
			} else chip->cur_period = 0;

			addr = (u_int32_t) chip->playback_substream->runtime->dma_area + (chip->cur_period * chip->period_size);

			flush_dcache_range(addr, addr + chip->period_size);
			LoadSample(addr, chip->period_size);

			StartSample();
			/* chip->start_play = 1; */
			
			snd_pcm_period_elapsed(chip->playback_substream);
		}
		AUDIO_DSP_CONTROL = (val | 0x100); /* clear DSP interrupt */
	}

	return IRQ_HANDLED;
}

static snd_pcm_ops_t snd_gamecube_playback_ops = {
	.open			= snd_gamecube_open,
	.close			= snd_gamecube_close,
	.ioctl			= snd_pcm_lib_ioctl,
	.hw_params	        = snd_gamecube_hw_params,
	.hw_free	        = snd_gamecube_hw_free,
	.prepare		= snd_gamecube_prepare,
	.trigger		= snd_gamecube_trigger,
	.pointer		= snd_gamecube_pointer,
};

static int __devinit snd_gamecube_new_pcm(gamecube_t *chip)
{
	snd_pcm_t *pcm;
	int err;

	if ((err = snd_pcm_new(chip->card, chip->card->shortname, 0, 1, 1, &pcm)) < 0)
		return err;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_gamecube_playback_ops);
	/* snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_gamecube_capture_ops); */
	
	/* preallocate 64k buffer */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_KERNEL),
			64 * 1024, 64 * 1024);

	pcm->info_flags = 0;
	pcm->private_data = chip;
	strcpy(pcm->name, chip->card->shortname);
	
	chip->pcm = pcm;

	return 0;
}

static int __init alsa_card_gamecube_init(void)
{
	int err;
	snd_card_t *card;

/*	if (!is_gamecube())
		return -ENODEV; */

	/* register the soundcard */
	card = snd_card_new(index, id, THIS_MODULE, sizeof(gamecube_t));
	if (card == NULL)
		return -ENOMEM;

	gamecube_audio = (gamecube_t *)card->private_data;
	if (gamecube_audio == NULL)
		return -ENOMEM;

	memset(gamecube_audio, 0, sizeof(gamecube_t));
	gamecube_audio->card = card;
	gamecube_audio->stop_play = 1;
	
	strcpy(card->driver, "GAMECUBE");
	strcpy(card->shortname, "GameCube audio");
	sprintf(card->longname, "GameCube audio");

	if (request_irq(DSP_IRQ, snd_gamecube_interrupt, 0, card->shortname, gamecube_audio)) {
		snd_printk(KERN_ERR "%s: unable to grab IRQ %d\n", card->shortname, DSP_IRQ);
		return -EBUSY;
	}

#if 0
	if (request_region(AUDIO_INTERFACE_ADDR, 0x200, card->shortname) == NULL) {
		printk("unable to grab memory region 0x%lx-0x%lx\n",
				AUDIO_INTERFACE_ADDR, AUDIO_INTERFACE_ADDR + 0x200 - 1);
		return -EBUSY;
	}

	if ((iobase = (unsigned long) ioremap(AUDIO_INTERFACE_ADDR, 0x200)) == 0) {
		printk("unable to remap memory region 0x%lx-0x%lx\n",
				AUDIO_INTERFACE_ADDR, AUDIO_INTERFACE_ADDR + 0x200 - 1);
		return -ENOMEM;
	}
		
	printk("iobase=0x%lx\n", iobase);
#endif

#if 0
	/* mixer */
	if ((err = snd_gamecube_mixer_new(gamecube_audio)) < 0)
		goto fail;
#endif
	/* PCM */
	if ((err = snd_gamecube_new_pcm(gamecube_audio)) < 0)
		goto fail;
        
	if ((err = snd_card_register(card)) == 0) {
		printk( KERN_INFO "GameCube audio support initialized\n" );
		return 0;
	}
	
fail:
	snd_card_free(card);
	return err;
}

static void __exit alsa_card_gamecube_exit(void)
{
	printk(KERN_ALERT "Goodbye, cruel world\n");

	free_irq(DSP_IRQ, gamecube_audio);
	snd_card_free(gamecube_audio->card);
}

module_init(alsa_card_gamecube_init)
module_exit(alsa_card_gamecube_exit)

