/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Brisk4t	
 *
 * BCM2835 I2S/PCM audio DAI driver for FreeBSD.
 * Based on rk_i2s.c by Oleksandr Tymoshenko <gonzo@FreeBSD.org>
 *
 * The bcm2835 DTS node does not carry an interrupt specifier by default.
 * A DTS overlay must add one before this driver can be used:
 *
 *   &i2s { interrupts = <0 119 4>; };  -- GIC SPI 119 (GPU IRQ 55, bank2 bit23 + 96)
 *
 * Clock management is delegated to bcm2835_clkman, which must be loaded
 * first (MODULE_DEPEND enforces this).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/resource.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"

#include <arm/broadcom/bcm2835/bcm2835_clkman.h>
#include "bcm2835_i2s.h"

static struct ofw_compat_data compat_data[] = {
	{ "brcm,bcm2835-i2s",	1 },
	{ NULL,			0 }
};

static struct resource_spec bcm2835_i2s_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

#define BCM2835_I2S_LOCK(sc)				mtx_lock(&(sc)->mtx)
#define BCM2835_I2S_UNLOCK(sc)				mtx_unlock(&(sc)->mtx)
#define BCM2835_I2S_READ_4(sc, reg)			bus_read_4((sc)->res[0], (reg))
#define BCM2835_I2S_WRITE_4(sc, reg, val) 	bus_write_4((sc)->res[0], (reg), (val))

/*
 * Wait for at least `cycles` PCM clocks using the CS_A SYNC echo mechanism.
 * Each toggle-and-poll iteration consumes exactly 2 PCM clocks, so odd values
 * are rounded up to the next even number.
 */
static void
bcm2835_i2s_sync_wait(struct bcm2835_i2s_softc *sc, int cycles)
{
	uint32_t cs, target;
	int i, n;

	n = (cycles + 1) / 2;
	for (i = 0; i < n; i++) {
		cs     = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
		target = (cs ^ CS_A_SYNC) & CS_A_SYNC;
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, (cs & ~CS_A_SYNC) | target);
		do {
			cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
		} while ((cs & CS_A_SYNC) != target);
	}
}

static uint32_t sc_fmt[] = {
	SND_FORMAT(AFMT_S8,     2, 0),
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	SND_FORMAT(AFMT_S32_LE, 2, 0),
	0
};

static struct pcmchan_caps bcm2835_i2s_caps = {
	BCM2835_I2S_RATE_MIN, BCM2835_I2S_RATE_MAX, sc_fmt, 0
};

/*
 * Pack bps bytes from a circular buffer starting at pos into a 32-bit
 * FIFO word (little-endian byte order, sample in the low bits).
 */
static inline uint32_t
bcm2835_i2s_pack_sample(const uint8_t *buf, uint32_t pos, uint32_t size,
    int bps)
{
	uint32_t v = 0;
	int i;

	for (i = 0; i < bps; i++)
		v |= (uint32_t)buf[(pos + i) % size] << (i * 8);
	return (v);
}

/*
 * Unpack a 32-bit FIFO word into bps bytes in a circular buffer.
 */
static inline void
bcm2835_i2s_unpack_sample(uint8_t *buf, uint32_t pos, uint32_t size,
    uint32_t val, int bps)
{
	int i;

	for (i = 0; i < bps; i++)
		buf[(pos + i) % size] = (val >> (i * 8)) & 0xff;
}

static int
bcm2835_i2s_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "BCM2835 I2S");
	return (BUS_PROBE_DEFAULT);
}

static int
bcm2835_i2s_detach(device_t dev)
{
	struct bcm2835_i2s_softc *sc;

	sc = device_get_softc(dev);

	if (sc->hw_started) {
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, 0);
		sc->hw_started = false;
	}

	if (sc->intrhand != NULL)
		bus_teardown_intr(dev, sc->res[1], sc->intrhand);

	bus_release_resources(dev, bcm2835_i2s_spec, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

static int
bcm2835_i2s_attach(device_t dev)
{
	struct bcm2835_i2s_softc *sc;
	phandle_t node;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* Allocate resources */
	if (bus_alloc_resources(dev, bcm2835_i2s_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources\n");
		error = ENXIO;
		goto fail;
	}

	/* Find the clock manager */
	sc->clkman = devclass_get_device(devclass_find("bcm2835_clkman"), 0);
	if (sc->clkman == NULL) {
		device_printf(dev, "cannot find bcm2835_clkman\n");
		error = ENXIO;
		goto fail;
	}

	/* Set PCM clock to a known-good default; set_chanspeed will reprogram it. */
	if (bcm2835_clkman_set_frequency(sc->clkman, BCM_PCM_CLKSRC,
	    BCM2835_I2S_RATE_DEFAULT * BCM2835_I2S_FRAME_LEN) == 0) {
		device_printf(dev, "cannot set PCM clock\n");
		error = ENXIO;
		goto fail;
	}
	sc->sample_rate    = BCM2835_I2S_RATE_DEFAULT;
	sc->txthr          = 2;
	sc->rxthr          = 1;
	sc->frame_len      = BCM2835_I2S_FRAME_LEN;
	sc->ch_width       = BCM2835_I2S_CHWIDTH;
	sc->bytes_per_sample = BCM2835_I2S_CHWIDTH / 8;
	sc->dai_fmt        = AUDIO_DAI_FORMAT_I2S;
	sc->packed_mode    = (BCM2835_I2S_CHWIDTH <= 16);

	sc->hw_started = true;

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);

fail:
	bcm2835_i2s_detach(dev);
	return (error);
}


static int
bcm2835_i2s_dai_init(device_t dev, uint32_t format)
{
	struct bcm2835_i2s_softc *sc;
	uint32_t mode, chc;
	int fmt, pol, clk;
	int ch1pos, ch2pos, flen, fslen;

	sc = device_get_softc(dev);

	fmt = AUDIO_DAI_FORMAT_FORMAT(format);
	pol = AUDIO_DAI_FORMAT_POLARITY(format);
	clk = AUDIO_DAI_FORMAT_CLOCK(format);

	mode = 0;

	switch (clk) {
	case AUDIO_DAI_CLOCK_CBM_CFM:
		/* BCM2835 drives BCLK and LRCLK (master mode). */
		break;
	case AUDIO_DAI_CLOCK_CBS_CFS:
		/* Codec drives BCLK and LRCLK; BCM2835 is clock slave. */
		mode |= MODE_A_CLKM | MODE_A_FSM;
		break;
	default:
		return (EINVAL);
	}

	/*
	 * Compute initial frame geometry using the default channel width.
	 * set_chanformat will recalculate when the PCM format is known.
	 * Frame length = 2 × ch_width (tight packing, one slot per channel).
	 */
	flen = 2 * sc->ch_width;

	switch (fmt) {
		case AUDIO_DAI_FORMAT_I2S:
			/*
			 * Standard I2S: FS high selects right channel.  Invert so
			 * FS high selects left (CH1).  CH1 data begins one BCLK
			 * after the FS edge per the BCM2835 convention.
			 */
			mode |= MODE_A_FSI;
			fslen  = sc->ch_width;
			ch1pos = 1;
			ch2pos = sc->ch_width + 1;
			break;
		case AUDIO_DAI_FORMAT_LJ:
			fslen  = sc->ch_width;
			ch1pos = 0;
			ch2pos = sc->ch_width;
			break;
		case AUDIO_DAI_FORMAT_RJ:
			/* Tight packing: slot_width == ch_width, so same as LJ. */
			fslen  = sc->ch_width;
			ch1pos = 0;
			ch2pos = sc->ch_width;
			break;
		case AUDIO_DAI_FORMAT_DSPA:
			mode |= MODE_A_FSI;
			fslen  = 1;
			ch1pos = 1;
			ch2pos = sc->ch_width + 1;
			break;
		case AUDIO_DAI_FORMAT_DSPB:
			fslen  = 1;
			ch1pos = 0;
			ch2pos = sc->ch_width;
			break;
		default:
			return (EINVAL);
	}

	if (AUDIO_DAI_POLARITY_INVERTED_BCLK(pol))
		mode ^= MODE_A_CLKI;
	if (AUDIO_DAI_POLARITY_INVERTED_FRAME(pol))
		mode ^= MODE_A_FSI;

	mode |= MODE_A_FLEN(flen - 1) | MODE_A_FSLEN(fslen);

	/* Store DAI format and initial frame geometry for set_chanformat. */
	sc->dai_fmt  = fmt;
	sc->frame_len = flen;

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, CS_A_EN); /* Enable PCM Block */

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_MODE_A, mode); /* Set the mode */

	chc = CHxC_CH1EN | CHxC_CH1POS(ch1pos) |
	      CHxC_CH1WID(sc->ch_width - 8) |
	      CHxC_CH2EN | CHxC_CH2POS(ch2pos) |
	      CHxC_CH2WID(sc->ch_width - 8);

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_TXC_A, chc);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_RXC_A, chc);

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, 0);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A,
	    INTx_A_RXERR | INTx_A_TXERR | INTx_A_RXR | INTx_A_TXW);

	return (0);
}

static int
bcm2835_i2s_dai_intr(device_t dev, struct snd_dbuf *play_buf,
    struct snd_dbuf *rec_buf)
{
	struct bcm2835_i2s_softc *sc;
	uint32_t intstc, cs, val;
	int ret = 0;

	sc = device_get_softc(dev);

	BCM2835_I2S_LOCK(sc);

	/* Read and clear the interrupt status */
	intstc = BCM2835_I2S_READ_4(sc, BCM_I2S_INTSTC_A);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, intstc);

	/* If a TXW interrupt occurred */
	if (intstc & INTx_A_TXW) {
		uint8_t *samples;
		uint32_t count, size, readyptr, written;
		int bps, frame_bytes;

		bps         = sc->bytes_per_sample;
		frame_bytes = bps * 2; /* one stereo frame */
		count    = sndbuf_getready(play_buf);
		size     = play_buf->bufsize;
		readyptr = sndbuf_getreadyptr(play_buf);
		samples  = play_buf->buf;
		written  = 0;

		if (sc->packed_mode) {
			/*
			 * FTXP=1: one 32-bit FIFO word carries both channels.
			 * L sample in [15:0], R sample in [31:16].
			 * Check TXD once per stereo frame.
			 */
			while (count >= (uint32_t)frame_bytes) {
				cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
				if (!(cs & CS_A_TXD))
					break;

				BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A,
				    bcm2835_i2s_pack_sample(samples,
				        readyptr, size, bps) |
				    (bcm2835_i2s_pack_sample(samples,
				        readyptr + bps, size, bps) << 16));
				readyptr += frame_bytes;

				written += frame_bytes;
				count   -= frame_bytes;
			}
		} else {
			/*
			 * FTXP=0: each FIFO word carries one channel.
			 * Write L then R, checking TXD once per pair to avoid
			 * splitting a stereo frame across a FIFO-full boundary.
			 */
			while (count >= (uint32_t)frame_bytes) {
				cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
				if (!(cs & CS_A_TXD))
					break;

				BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A,
				    bcm2835_i2s_pack_sample(samples,
				        readyptr, size, bps));
				readyptr += bps;

				BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A,
				    bcm2835_i2s_pack_sample(samples,
				        readyptr, size, bps));
				readyptr += bps;

				written += frame_bytes;
				count   -= frame_bytes;
			}
		}

		sc->play_ptr += written;
		sc->play_ptr %= size;
		ret |= AUDIO_DAI_PLAY_INTR;
	}

	/* If an RXR interrupt occurred */
	if (intstc & INTx_A_RXR) {
		uint8_t *samples;
		uint32_t count, size, freeptr, recorded;
		int bps, frame_bytes;

		bps         = sc->bytes_per_sample;
		frame_bytes = bps * 2; /* one stereo frame */
		count    = sndbuf_getfree(rec_buf);
		size     = rec_buf->bufsize;
		freeptr  = sndbuf_getfreeptr(rec_buf);
		samples  = rec_buf->buf;
		recorded = 0;

		if (sc->packed_mode) {
			/*
			 * FRXP=1: one 32-bit FIFO word carries both channels.
			 * L sample in [15:0], R sample in [31:16].
			 */
			while (count >= (uint32_t)frame_bytes) {
				cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
				if (!(cs & CS_A_RXD))
					break;

				val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);
				bcm2835_i2s_unpack_sample(samples, freeptr,
				    size, val, bps);               /* L */
				bcm2835_i2s_unpack_sample(samples, freeptr + bps,
				    size, val >> 16, bps);         /* R */
				freeptr += frame_bytes;

				recorded += frame_bytes;
				count    -= frame_bytes;
			}
		} else {
			/* FRXP=0: each FIFO word carries one channel. */
			while (count >= (uint32_t)frame_bytes) {
				cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
				if (!(cs & CS_A_RXD))
					break;

				val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);
				bcm2835_i2s_unpack_sample(samples, freeptr,
				    size, val, bps);
				freeptr += bps;

				val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);
				bcm2835_i2s_unpack_sample(samples, freeptr,
				    size, val, bps);
				freeptr += bps;

				recorded += frame_bytes;
				count    -= frame_bytes;
			}
		}

		sc->rec_ptr += recorded;
		sc->rec_ptr %= size;
		ret |= AUDIO_DAI_REC_INTR;
	}

	BCM2835_I2S_UNLOCK(sc);

	return (ret);
}

static struct pcmchan_caps *
bcm2835_i2s_dai_get_caps(device_t dev)
{
	return (&bcm2835_i2s_caps);
}

static int
bcm2835_i2s_dai_trigger(device_t dev, int go, int pcm_dir)
{
	struct bcm2835_i2s_softc *sc = device_get_softc(dev);
	uint32_t cs, inten;

	if ((pcm_dir != PCMDIR_PLAY) && (pcm_dir != PCMDIR_REC))
		return (EINVAL);

	switch (go) {
		case PCMTRIG_START:
			cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
			if (pcm_dir == PCMDIR_PLAY) {
				
				/* Clear the TX FIFO */
				BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, cs | CS_A_TXCLR);
				bcm2835_i2s_sync_wait(sc, 2); /* Wait for 2 PCM clocks */

				/* Set interrupts to fire when FIFO level is lower than TXTHR level */
				inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
				BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, inten | INTx_A_TXW);
				
				/* Silence Pre-fill */
				for (int i = 0; i < 64; i++){
					BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A, 0);
				}

				/* Clear the TXW interrupt status */
				BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, INTx_A_TXW);

				BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
				    cs | CS_A_TXON |
				    CS_A_TXTHR(sc->txthr) | CS_A_RXTHR(sc->rxthr));

			} 
			
			else {

				BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, INTx_A_RXR);
				
				inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
				BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, inten | INTx_A_RXR);
				
				BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
				    cs | CS_A_EN | CS_A_RXON | CS_A_RXCLR |
				    CS_A_RXTHR(sc->rxthr));
			}
			break;

		case PCMTRIG_STOP:
		case PCMTRIG_ABORT:
			/* Stop PCM first; INTEN_A must not be written while running. */
			cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
			if (pcm_dir == PCMDIR_PLAY) {
				cs &= ~CS_A_TXON;
				cs |= CS_A_TXCLR;
			} else {
				cs &= ~CS_A_RXON;
				cs |= CS_A_RXCLR;
			}
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, cs);

			inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
			if (pcm_dir == PCMDIR_PLAY)
				inten &= ~INTx_A_TXW;
			else
				inten &= ~INTx_A_RXR;
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, inten);

			BCM2835_I2S_LOCK(sc);
			if (pcm_dir == PCMDIR_PLAY)
				sc->play_ptr = 0;
			else
				sc->rec_ptr = 0;
			BCM2835_I2S_UNLOCK(sc);
			break;
	}

	return (0);
}

static uint32_t
bcm2835_i2s_dai_get_ptr(device_t dev, int pcm_dir)
{
	struct bcm2835_i2s_softc *sc;
	uint32_t ptr;

	sc = device_get_softc(dev);

	BCM2835_I2S_LOCK(sc);
	ptr = (pcm_dir == PCMDIR_PLAY) ? sc->play_ptr : sc->rec_ptr;
	BCM2835_I2S_UNLOCK(sc);

	return (ptr);
}

static int
bcm2835_i2s_dai_setup_intr(device_t dev, driver_intr_t intr_handler,
    void *intr_arg)
{
	struct bcm2835_i2s_softc *sc = device_get_softc(dev);

	if (bus_setup_intr(dev, sc->res[1],
	    INTR_TYPE_AV | INTR_MPSAFE, NULL, intr_handler, intr_arg,
	    &sc->intrhand)) {
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	return (0);
}

static uint32_t
bcm2835_i2s_dai_set_chanformat(device_t dev, uint32_t format)
{
	struct bcm2835_i2s_softc *sc;
	uint32_t chc, mode;
	int ch_width, bps, frame_len, fslen, ch1pos, ch2pos, wid;
	bool wex;

	sc = device_get_softc(dev);

	ch_width = AFMT_BIT(format);
	bps      = AFMT_BPS(format);

	/*
	 * Frame = 2 × ch_width BCLK (tight packing: one slot per channel,
	 * slot width equals sample width).  This minimises BCLK frequency and
	 * allows feeder_rate to return an accurate achieved sample rate.
	 */
	frame_len = 2 * ch_width;

	/*
	 * Channel width register encoding:
	 *   WEX=0 → actual width = WID + 8   (covers  8–23 bit)
	 *   WEX=1 → actual width = WID + 24  (covers 24–39 bit)
	 * CHxC_CH1WID(n) stores n = width – 8 masked to 4 bits, which
	 * naturally gives the correct WID value in both cases:
	 *   16-bit: (16-8)=8  & 0xf = 8,  WEX=0 → 8+8=16  ✓
	 *   32-bit: (32-8)=24 & 0xf = 8,  WEX=1 → 8+24=32 ✓
	 *   24-bit: (24-8)=16 & 0xf = 0,  WEX=1 → 0+24=24 ✓
	 *    8-bit: (8-8) =0  & 0xf = 0,  WEX=0 → 0+8=8   ✓
	 */
	wid = ch_width - 8;
	wex = (ch_width > 23);

	/* Channel positions depend on the DAI format stored at dai_init time. */
	switch (sc->dai_fmt) {
	case AUDIO_DAI_FORMAT_I2S:
		/* One BCLK delay after FS edge (I2S spec). */
		fslen  = ch_width;
		ch1pos = 1;
		ch2pos = ch_width + 1;
		break;
	case AUDIO_DAI_FORMAT_LJ:
		fslen  = ch_width;
		ch1pos = 0;
		ch2pos = ch_width;
		break;
	case AUDIO_DAI_FORMAT_RJ:
		/* Tight packing: slot_width == ch_width → equivalent to LJ. */
		fslen  = ch_width;
		ch1pos = 0;
		ch2pos = ch_width;
		break;
	case AUDIO_DAI_FORMAT_DSPA:
		fslen  = 1;
		ch1pos = 1;
		ch2pos = ch_width + 1;
		break;
	case AUDIO_DAI_FORMAT_DSPB:
		fslen  = 1;
		ch1pos = 0;
		ch2pos = ch_width;
		break;
	default:
		return (0);
	}

	/*
	 * Packed frame mode (FTXP/FRXP): the hardware merges both stereo
	 * channels into one 32-bit FIFO word (CH1 in [15:0], CH2 in [31:16]).
	 * This halves FIFO traffic but is limited to ch_width <= 16 bits.
	 */
	bool packed = (ch_width <= 16);

	/*
	 * Update frame geometry and packed-mode bits in MODE_A, preserving
	 * the clock and polarity bits (CLKM, CLKI, FSM, FSI, CLK_DIS).
	 */
	mode = BCM2835_I2S_READ_4(sc, BCM_I2S_MODE_A);
	mode &= ~(MODE_A_FLEN(0x3ff) | MODE_A_FSLEN(0x3ff) |
	    MODE_A_FTXP | MODE_A_FRXP);
	mode |= MODE_A_FLEN(frame_len - 1) | MODE_A_FSLEN(fslen);
	if (packed)
		mode |= MODE_A_FTXP | MODE_A_FRXP;
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_MODE_A, mode);

	chc = (wex ? CHxC_CH1WEX : 0) |
	    CHxC_CH1EN | CHxC_CH1POS(ch1pos) | CHxC_CH1WID(wid) |
	    (wex ? CHxC_CH2WEX : 0) |
	    CHxC_CH2EN | CHxC_CH2POS(ch2pos) | CHxC_CH2WID(wid);

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_TXC_A, chc);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_RXC_A, chc);

	BCM2835_I2S_LOCK(sc);
	sc->frame_len        = frame_len;
	sc->ch_width         = ch_width;
	sc->bytes_per_sample = bps;
	sc->packed_mode      = packed;
	BCM2835_I2S_UNLOCK(sc);

	return (0);
}

static int
bcm2835_i2s_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{
	struct bcm2835_i2s_softc *sc;

	sc = device_get_softc(dev);
	if (sc->clkman == NULL)
		return (ENXIO);

	if (bcm2835_clkman_set_frequency(sc->clkman, BCM_PCM_CLKSRC,
	    rate) == 0) {
		device_printf(sc->dev, "could not set PCM clock to %u Hz\n",
		    rate);
		return (EIO);
	}

	return (0);
}

static uint32_t
bcm2835_i2s_dai_set_chanspeed(device_t dev, uint32_t speed)
{
	struct bcm2835_i2s_softc *sc;
	uint32_t mode, actual_bclk, actual_rate;

	sc = device_get_softc(dev);
	if (sc->clkman == NULL)
		return (speed);

	/*
	 * TXTHR=1: TXW fires when TX FIFO < 3/4 full (16 words free, ~1ms at 8kHz).
	 * TXTHR=2: TXW fires when TX FIFO < 1/2 full (32 words free, ~333us at 48kHz).
	 * Higher rates drain the FIFO faster so we fire earlier to avoid underrun.
	 * RXTHR=1: RXR fires when RX FIFO >= 1/4 full (16 words), leaving 48 words
	 * of headroom before overflow at any supported rate.
	 */
	sc->txthr = (speed > 16000) ? 2 : 1;
	sc->rxthr = 1;

	mode = BCM2835_I2S_READ_4(sc, BCM_I2S_MODE_A);
	if (mode & MODE_A_CLKM) {
		/* Clock slave: external BCLK drives rate, nothing to program. */
		sc->sample_rate = speed;
		return (speed);
	}

	actual_bclk = bcm2835_clkman_set_frequency(sc->clkman, BCM_PCM_CLKSRC,
	    speed * sc->frame_len);
	if (actual_bclk == 0) {
		device_printf(sc->dev, "could not set PCM clock for %u Hz\n",
		    speed);
		return (speed);
	}

	/*
	 * Return the actual achieved rate so the PCM layer can insert
	 * feeder_rate to compensate for clock-divider rounding.
	 */
	actual_rate = actual_bclk / sc->frame_len;
	sc->sample_rate = actual_rate;
	return (actual_rate);
}

static device_method_t bcm2835_i2s_methods[] = {
	DEVMETHOD(device_probe,			bcm2835_i2s_probe),
	DEVMETHOD(device_attach,		bcm2835_i2s_attach),
	DEVMETHOD(device_detach,		bcm2835_i2s_detach),

	DEVMETHOD(audio_dai_init,		bcm2835_i2s_dai_init),
	DEVMETHOD(audio_dai_setup_intr,		bcm2835_i2s_dai_setup_intr),
	DEVMETHOD(audio_dai_set_sysclk,		bcm2835_i2s_dai_set_sysclk),
	DEVMETHOD(audio_dai_set_chanspeed,	bcm2835_i2s_dai_set_chanspeed),
	DEVMETHOD(audio_dai_set_chanformat,	bcm2835_i2s_dai_set_chanformat),
	DEVMETHOD(audio_dai_intr,		bcm2835_i2s_dai_intr),
	DEVMETHOD(audio_dai_get_caps,		bcm2835_i2s_dai_get_caps),
	DEVMETHOD(audio_dai_trigger,		bcm2835_i2s_dai_trigger),
	DEVMETHOD(audio_dai_get_ptr,		bcm2835_i2s_dai_get_ptr),

	DEVMETHOD_END
};

static driver_t bcm2835_i2s_driver = {
	"i2s",
	bcm2835_i2s_methods,
	sizeof(struct bcm2835_i2s_softc),
};

DRIVER_MODULE(bcm2835_i2s, simplebus, bcm2835_i2s_driver, 0, 0);
MODULE_DEPEND(bcm2835_i2s, bcm2835_clkman, 1, 1, 1);
SIMPLEBUS_PNP_INFO(compat_data);
