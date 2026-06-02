/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 Brisk4t
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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
#include <arm/broadcom/bcm2835/bcm2835_clkman.h>
#include "audio_dai_if.h"

#include "bcm2835_i2s.h"

static struct ofw_compat_data compat_data[] = {
	{ "brcm,bcm2835-i2s",	1 },
	{ "brcm,bcm2711-i2s",	1 },
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
	SND_FORMAT(AFMT_S8,     1, 0),
	SND_FORMAT(AFMT_S8,     2, 0),
	SND_FORMAT(AFMT_S16_LE, 1, 0),
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	SND_FORMAT(AFMT_S24_LE, 1, 0),
	SND_FORMAT(AFMT_S24_LE, 2, 0),
	SND_FORMAT(AFMT_S32_LE, 1, 0),
	SND_FORMAT(AFMT_S32_LE, 2, 0),
	0
};

static struct pcmchan_caps bcm2835_i2s_caps = {
	BCM2835_I2S_RATE_MIN, BCM2835_I2S_RATE_MAX, sc_fmt, 0
};

/*
 * Pack bps bytes from the PCM ring buffer into a 32-bit FIFO word.
 * Bytes are assembled little-endian (byte 0 → bits [7:0]).  The modulo
 * wrap handles the rare case where a sample straddles the ring boundary.
 */
static inline uint32_t
bcm2835_i2s_assemble_sample(const uint8_t *buf, uint32_t pos, uint32_t size,
    int bps)
{
	uint32_t v = 0;
	int i;

	for (i = 0; i < bps; i++)
		v |= (uint32_t)buf[(pos + i) % size] << (i * 8);
	return (v);
}

/*
 * Scatter a 32-bit FIFO word into bps little-endian bytes in the PCM ring buffer.
 */
static inline void
bcm2835_i2s_scatter_sample(uint8_t *buf, uint32_t pos, uint32_t size,
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

	/* Disable the PCM block on detach */
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

	sc->sample_rate      = BCM2835_I2S_RATE_DEFAULT;
	sc->txthr            = 2;
	sc->rxthr            = 1;
	sc->frame_len        = BCM2835_I2S_FRAME_LEN;
	sc->ch_width         = BCM2835_I2S_CHWIDTH;
	sc->bytes_per_sample = BCM2835_I2S_CHWIDTH / 8;
	sc->num_channels     = 2;
	sc->dai_fmt          = AUDIO_DAI_FORMAT_I2S;
	sc->packed_mode      = (BCM2835_I2S_CHWIDTH <= 16);

	/* Cleanly detach */
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
	flen = 2 * sc->ch_width;

	switch (clk) {
		case AUDIO_DAI_CLOCK_CBM_CFM:
			break;
		case AUDIO_DAI_CLOCK_CBS_CFM:
			mode |= MODE_A_CLKM;
			break;
		case AUDIO_DAI_CLOCK_CBM_CFS:
			mode |= MODE_A_FSM;
			break;
		case AUDIO_DAI_CLOCK_CBS_CFS:
			mode |= MODE_A_CLKM | MODE_A_FSM;
			break;
		default:
			return (EINVAL);
	}
	
	switch (fmt) {
	case AUDIO_DAI_FORMAT_I2S:
		/*
		 * BCM2835 §8.2: FS is HIGH during the first half-frame (CH1 =
		 * left channel) by default.  Modern codecs (PCM5102, WM8731,
		 * …) use LRCLK=HIGH=LEFT, so no inversion is needed here.
		 * Codecs that follow the original Philips convention
		 * (WS=0=LEFT) must set frame-inversion polarity in their DTS
		 * node; the MODE_A_FSI toggle below handles that case.
		 * CH1 data begins one BCLK after the FS edge (I2S delay).
		 */
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
		/*
		 * Right-justified within the slot.  With tight packing
		 * (slot_width == ch_width) this is identical to LJ.
		 * set_chanformat recomputes once the actual sample width
		 * is known, placing samples at the right edge of each slot.
		 */
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

	sc->dai_fmt   = fmt;
	sc->frame_len = flen;

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, CS_A_EN); /* Enable PCM Block */

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_MODE_A, mode); /* Set the mode */

	chc = CHxC_CH1EN | CHxC_CH1POS(ch1pos) |
	      CHxC_CH1WID(sc->ch_width - 8) |
	      CHxC_CH2EN | CHxC_CH2POS(ch2pos) |
	      CHxC_CH2WID(sc->ch_width - 8);

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_TXC_A, chc);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_RXC_A, chc);

	/* Step 3 */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, 0);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, INTx_A_ALL);

	/*
	 * DREQ thresholds: TX panic at ≤16 words (empty quarter), normal at
	 * ≤16 words; RX panic at ≥48 words (full three-quarters), normal at
	 * ≥48 words.  These apply when DMA is enabled (CS_A_DMAEN); the IRQ
	 * path uses TXTHR/RXTHR instead but setting DREQ_A costs nothing.
	 */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_DREQ_A,
	    DREQ_A_TX_PANIC(0x10) | DREQ_A_RX_PANIC(0x30) |
	    DREQ_A_TX(0x10)       | DREQ_A_RX(0x30));

	/* Release standby, then wait ≥4 PCM clocks. */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, CS_A_EN | CS_A_STBY);
	bcm2835_i2s_sync_wait(sc, 4);

	/* Step 6: flush FIFOs; RXSEX sign-extends narrow RX samples to 32 b. */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
	    CS_A_EN | CS_A_STBY | CS_A_TXCLR | CS_A_RXCLR | CS_A_RXSEX);

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

	/* FIFO error recovery: flush the offending FIFO and clear the latch. */
	if (intstc & (INTx_A_TXERR | INTx_A_RXERR)) {
		uint32_t cs_err = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
		if (intstc & INTx_A_TXERR)
			cs_err |= CS_A_TXCLR;
		if (intstc & INTx_A_RXERR)
			cs_err |= CS_A_RXCLR;
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, cs_err);
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A,
		    intstc & (INTx_A_TXERR | INTx_A_RXERR));
	}

	/* If a TXW interrupt occurred */
	if (intstc & INTx_A_TXW) {
		uint8_t *samples;
		uint32_t count, size, readyptr, written;
		int bps, frame_bytes;

		bps         = sc->bytes_per_sample;
		frame_bytes = bps * sc->num_channels;
		count    = sndbuf_getready(play_buf);
		size     = play_buf->bufsize;
		readyptr = sndbuf_getreadyptr(play_buf);
		samples  = play_buf->buf;
		written  = 0;

		if (sc->packed_mode) {
			/*
			 * FTXP=1: one 32-bit FIFO word carries both channels.
			 * The BCM2835 shifts PCM bits MSB first, so channel 1
			 * occupies the lower half-word and channel 2 the upper
			 * half-word per the FIFO packing rules.
			 * Check TXD once per stereo frame.
			 */
			while (count >= (uint32_t)frame_bytes) {
				cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
				if (!(cs & CS_A_TXD))
					break;

				BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A,
				    bcm2835_i2s_assemble_sample(samples,
				        readyptr, size, bps) |
				    (bcm2835_i2s_assemble_sample(samples,
				        readyptr + bps, size, bps) << 16));
				readyptr += frame_bytes;

				written += frame_bytes;
				count   -= frame_bytes;
			}
		} 
		
		else {
			/*
			 * FTXP=0: one FIFO word per enabled channel.
			 * Mono: write CH1 only (CH2 disabled in TXC_A).
			 * Stereo: write CH1 then CH2 in one TXD check to avoid
			 * splitting a frame across a FIFO-full boundary.
			 */
			while (count >= (uint32_t)frame_bytes) {
				cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
				if (!(cs & CS_A_TXD))
					break;

				BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A,
				    bcm2835_i2s_assemble_sample(samples,
				        readyptr, size, bps));
				readyptr += bps;

				if (sc->num_channels == 2) {
					BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A,
					    bcm2835_i2s_assemble_sample(samples,
					        readyptr, size, bps));
					readyptr += bps;
				}

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
		frame_bytes = bps * sc->num_channels;
		count    = sndbuf_getfree(rec_buf);
		size     = rec_buf->bufsize;
		freeptr  = sndbuf_getfreeptr(rec_buf);
		samples  = rec_buf->buf;
		recorded = 0;

		if (sc->packed_mode) {
			/*
			 * FRXP=1: one 32-bit FIFO word carries both channels.
			 * Channel 1 is stored in the lower half-word and channel 2
			 * in the upper half-word per the FIFO packing rules.
			 */
			while (count >= (uint32_t)frame_bytes) {
				cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
				if (!(cs & CS_A_RXD))
					break;

				val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);
				bcm2835_i2s_scatter_sample(samples, freeptr,
				    size, val, bps);               /* L */
				bcm2835_i2s_scatter_sample(samples, freeptr + bps,
				    size, val >> 16, bps);         /* R */
				freeptr += frame_bytes;

				recorded += frame_bytes;
				count    -= frame_bytes;
			}
		} else { // TODO: Potentially incorrect else implementation
			/* FRXP=0: one FIFO word per enabled channel. */
			while (count >= (uint32_t)frame_bytes) {
				cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
				if (!(cs & CS_A_RXD))
					break;

				val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);
				bcm2835_i2s_scatter_sample(samples, freeptr,
				    size, val, bps);
				freeptr += bps;

				if (sc->num_channels == 2) {
					val = BCM2835_I2S_READ_4(sc,
					    BCM_I2S_FIFO_A);
					bcm2835_i2s_scatter_sample(samples,
					    freeptr, size, val, bps);
					freeptr += bps;
				}

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
				if (cs & CS_A_TXON) {
					/*
					 * Already transmitting: the PCM layer re-triggers
					 * after each ISR callback.  Just keep the interrupt
					 * armed and return — dont clear the FIFO or
					 * re-prefill with silence, which would glitch audio.
					 */
					inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
					BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A,
					    inten | INTx_A_TXW);
					break;
				}

				// TODO: Prefill with known audio data instead of silence
				/* First start: clear FIFO, prefill silence, then enable TX. */
				BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, cs | CS_A_TXCLR);
				bcm2835_i2s_sync_wait(sc, 2);

				inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
				BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, inten | INTx_A_TXW);

				for (int i = 0; i < 64; i++)
					BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A, 0);

				BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, INTx_A_TXW);

				BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
				    cs | CS_A_TXON |
				    CS_A_TXTHR(sc->txthr) | CS_A_RXTHR(sc->rxthr));
			} else {
				if (cs & CS_A_RXON) {
					inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
					BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A,
					    inten | INTx_A_RXR);
					break;
				}

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
	uint32_t chc_tx, chc_rx, mode;
	int ch_width, bps, num_ch, frame_len, fslen, ch1pos, ch2pos, wid;
	bool wex, packed;

	sc = device_get_softc(dev);

	ch_width = AFMT_BIT(format);
	bps      = AFMT_BPS(format);
	num_ch   = AFMT_CHANNEL(format);	/* 1 = mono, 2 = stereo */

	/*
	 * Wire frame always spans two slots for I2S/LJ/RJ compatibility —
	 * FLEN covers both channels regardless of num_ch.  FIFO depth per
	 * frame is num_ch words (one per enabled channel).
	 */
	frame_len = 2 * ch_width;
	wid = ch_width - 8;
	wex = (ch_width > 23);

	switch (sc->dai_fmt) {
	case AUDIO_DAI_FORMAT_I2S:
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
		/*
		 * Right-justified: samples sit at the right edge of each slot.
		 * With tight packing slot_width == ch_width so positions
		 * coincide with LJ.  When a future DT slot-width property is
		 * added, ch1pos = slot_width - ch_width and
		 * ch2pos = 2*slot_width - ch_width.
		 */
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
	 * Packed mode (FTXP/FRXP): hardware merges both stereo channels into
	 * one 32-bit FIFO word (CH1 in [15:0], CH2 in [31:16]).  Only valid
	 * for stereo ≤16-bit; mono uses one unpacked FIFO word per frame.
	 */
	packed = (ch_width <= 16) && (num_ch == 2);

	mode = BCM2835_I2S_READ_4(sc, BCM_I2S_MODE_A);
	mode &= ~(MODE_A_FLEN(0x3ff) | MODE_A_FSLEN(0x3ff) |
	    MODE_A_FTXP | MODE_A_FRXP);
	mode |= MODE_A_FLEN(frame_len - 1) | MODE_A_FSLEN(fslen);
	if (packed)
		mode |= MODE_A_FTXP | MODE_A_FRXP;
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_MODE_A, mode);

	/*
	 * For stereo enable both channels; for mono enable CH1 only.
	 * CH2 disabled in mono: hardware outputs silence on the right slot
	 * (TX) and discards right-slot data (RX).
	 */
	chc_tx = (wex ? CHxC_CH1WEX : 0) |
	    CHxC_CH1EN | CHxC_CH1POS(ch1pos) | CHxC_CH1WID(wid);
	chc_rx = chc_tx;

	if (num_ch == 2) {
		chc_tx |= (wex ? CHxC_CH2WEX : 0) |
		    CHxC_CH2EN | CHxC_CH2POS(ch2pos) | CHxC_CH2WID(wid);
		chc_rx  = chc_tx;
	}

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_TXC_A, chc_tx);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_RXC_A, chc_rx);

	BCM2835_I2S_LOCK(sc);
	sc->frame_len        = frame_len;
	sc->ch_width         = ch_width;
	sc->bytes_per_sample = bps;
	sc->num_channels     = num_ch;
	sc->packed_mode      = packed;
	BCM2835_I2S_UNLOCK(sc);

	return (0);
}

static int
bcm2835_i2s_dai_set_sysclk(device_t dev __unused, unsigned int rate __unused,
    int dai_dir __unused)
{
	/*
	TODO: Look into what this is supposed to do
	 */
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


	sc->txthr = (speed > 16000) ? 2 : 1;
	sc->rxthr = 1;

	mode = BCM2835_I2S_READ_4(sc, BCM_I2S_MODE_A);
	if (mode & MODE_A_CLKM) {
		/* Clock slave: external BCLK drives rate, nothing to program. */
		sc->sample_rate = speed;
		return (speed);
	}

	actual_bclk = bcm2835_clkman_set_frequency(sc->clkman, BCM_PCM_CLKSRC, speed * sc->frame_len);
	if (actual_bclk == 0) {
		device_printf(sc->dev, "could not set PCM clock for %u Hz\n",
		    speed);
		return (speed);
	}

	actual_rate = actual_bclk / sc->frame_len;
	sc->sample_rate = actual_rate;
	return (actual_rate);
}

static device_method_t bcm2835_i2s_methods[] = {
	DEVMETHOD(device_probe,				bcm2835_i2s_probe),
	DEVMETHOD(device_attach,			bcm2835_i2s_attach),
	DEVMETHOD(device_detach,			bcm2835_i2s_detach),

	DEVMETHOD(audio_dai_init,			bcm2835_i2s_dai_init),
	DEVMETHOD(audio_dai_setup_intr,		bcm2835_i2s_dai_setup_intr),
	DEVMETHOD(audio_dai_set_sysclk,		bcm2835_i2s_dai_set_sysclk),
	DEVMETHOD(audio_dai_set_chanspeed,	bcm2835_i2s_dai_set_chanspeed),
	DEVMETHOD(audio_dai_set_chanformat,	bcm2835_i2s_dai_set_chanformat),
	DEVMETHOD(audio_dai_intr,			bcm2835_i2s_dai_intr),
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
