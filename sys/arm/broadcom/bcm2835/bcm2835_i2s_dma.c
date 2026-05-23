/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024
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

#include <arm/broadcom/bcm2835/bcm2835_dma.h>

#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include <arm/broadcom/bcm2835/bcm2835_clkman.h>

#include "opt_snd.h"
#include "audio_dai_if.h"


#include "bcm2835_i2s_dma.h"

static struct ofw_compat_data compat_data[] = {
	{ "brcm,bcm2835-i2s",	1 },
	{ NULL,			0 }
};

static struct resource_spec bcm2835_i2s_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

#define BCM2835_I2S_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define BCM2835_I2S_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define BCM2835_I2S_READ_4(sc, reg)	\
    bus_read_4((sc)->res[0], (reg))
#define BCM2835_I2S_WRITE_4(sc, reg, val) \
    bus_write_4((sc)->res[0], (reg), (val))

static uint32_t sc_fmt[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	0
};
static struct pcmchan_caps bcm2835_i2s_caps = {
	BCM2835_I2S_SAMPLING_RATE, BCM2835_I2S_SAMPLING_RATE, sc_fmt, 0
};

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

	if (sc->dma_chan_tx != BCM_DMA_CH_INVALID)
		bcm_dma_free(sc->dma_chan_tx);
	if (sc->dma_chan_rx != BCM_DMA_CH_INVALID)
		bcm_dma_free(sc->dma_chan_rx);

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

	if (bus_alloc_resources(dev, bcm2835_i2s_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources\n");
		error = ENXIO;
		goto fail;
	}

	/* ----------------- Clock Functions --------------------------- */
	/* Get clkmanager */
	sc->clkman = devclass_get_device(devclass_find("bcm2835_clkman"), 0);
	if (sc->clkman == NULL) {
		device_printf(dev, "cannot find bcm2835_clkman\n");
		error = ENXIO;
		goto fail;
	}

	/*
	 * Start the PCM clock before touching any I2S registers.  Without a
	 * running clock the APB bus stalls on the first register access,
	 * producing a synchronous data abort on arm64.  hw_started is set
	 * immediately after so that detach knows register accesses are safe
	 * if any later step fails.
	 *
	 * Full block initialisation (MODE_A, TXC_A, RXC_A, EN/STBY) is
	 * deferred to dai_init where the frame format is known.
	 */
	if (bcm2835_clkman_set_frequency(sc->clkman, BCM_PCM_CLKSRC,
	    BCM2835_I2S_SAMPLING_RATE * BCM2835_I2S_FRAME_LEN) == 0) {
		device_printf(dev, "cannot set PCM clock\n");
		error = ENXIO;
		goto fail;
	}
	sc->hw_started = true;

	/*
	 * Pre-initialise to BCM_DMA_CH_INVALID so that a partial failure
	 * (one channel allocated, the other not) is detectable.
	 */
	sc->dma_chan_rx = BCM_DMA_CH_INVALID;
	sc->dma_chan_tx = BCM_DMA_CH_INVALID;

	sc->dma_chan_rx = bcm_dma_allocate(BCM_DMA_CH_ANY);
	sc->dma_chan_tx = bcm_dma_allocate(BCM_DMA_CH_ANY);
	if (sc->dma_chan_rx == BCM_DMA_CH_INVALID ||
	    sc->dma_chan_tx == BCM_DMA_CH_INVALID) {
		device_printf(dev, "cannot allocate DMA channels\n");
		error = ENXIO;
		goto fail;
	}

	/* TX channel: system memory → I2S FIFO (DREQ-paced, fixed dest addr) */
	bcm_dma_setup_src(sc->dma_chan_tx, BCM_DMA_DREQ_NONE,
	    BCM_DMA_INC_ADDR, BCM_DMA_32BIT);
	bcm_dma_setup_dst(sc->dma_chan_tx, BCM2835_I2S_DREQ_TX,
	    BCM_DMA_SAME_ADDR, BCM_DMA_32BIT);

	/* RX channel: I2S FIFO → system memory (DREQ-paced, fixed src addr) */
	bcm_dma_setup_src(sc->dma_chan_rx, BCM2835_I2S_DREQ_RX,
	    BCM_DMA_SAME_ADDR, BCM_DMA_32BIT);
	bcm_dma_setup_dst(sc->dma_chan_rx, BCM_DMA_DREQ_NONE,
	    BCM_DMA_INC_ADDR, BCM_DMA_32BIT);

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

	flen = BCM2835_I2S_FRAME_LEN;

	switch (fmt) {
	case AUDIO_DAI_FORMAT_I2S:
		/*
		 * Standard I2S: FS high selects right channel.  Invert so
		 * FS high selects left (CH1).  CH1 data begins one BCLK
		 * after the FS edge per the BCM2835 convention.
		 */
		mode |= MODE_A_FSI;
		fslen = flen / 2;
		ch1pos = 1;
		ch2pos = flen / 2 + 1;
		break;
	case AUDIO_DAI_FORMAT_LJ:
		fslen = flen / 2;
		ch1pos = 0;
		ch2pos = flen / 2;
		break;
	case AUDIO_DAI_FORMAT_RJ:
		fslen = flen / 2;
		ch1pos = flen / 2 - BCM2835_I2S_CHWIDTH;
		ch2pos = flen - BCM2835_I2S_CHWIDTH;
		break;
	case AUDIO_DAI_FORMAT_DSPA:
		mode |= MODE_A_FSI;
		flen  = 2 * BCM2835_I2S_CHWIDTH;
		fslen = 1;
		ch1pos = 1;
		ch2pos = BCM2835_I2S_CHWIDTH + 1;
		break;
	case AUDIO_DAI_FORMAT_DSPB:
		flen  = 2 * BCM2835_I2S_CHWIDTH;
		fslen = 1;
		ch1pos = 0;
		ch2pos = BCM2835_I2S_CHWIDTH;
		break;
	default:
		return (EINVAL);
	}

	if (AUDIO_DAI_POLARITY_INVERTED_BCLK(pol))
		mode ^= MODE_A_CLKI;
	if (AUDIO_DAI_POLARITY_INVERTED_FRAME(pol))
		mode ^= MODE_A_FSI;

	mode |= MODE_A_FLEN(flen - 1) | MODE_A_FSLEN(fslen);

	/*
	 * BCM2835 ARM Peripherals §8.4 startup sequence:
	 *  1. Disable the PCM block (EN=0) before writing mode/channel regs.
	 *  2. Write MODE_A, TXC_A, RXC_A while the block is inactive.
	 *  3. Release standby (STBY=1); wait ≥4 PCM clocks via SYNC.
	 *  4. Enable the block (EN=1).
	 *  5. Flush FIFOs, clear interrupt flags, set thresholds.
	 */

	// Disable PCM block before configuring
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, 0);

	// Set mode register
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_MODE_A, mode);


	chc = CHxC_CH1EN | CHxC_CH1POS(ch1pos) |
	      CHxC_CH1WID(BCM2835_I2S_CHWIDTH - 8) |
	      CHxC_CH2EN | CHxC_CH2POS(ch2pos) |
	      CHxC_CH2WID(BCM2835_I2S_CHWIDTH - 8);

	// RX and TX have the same channel config for now
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_TXC_A, chc);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_RXC_A, chc);
		
	/*
	 * BCM2835 §CS_A note: only the bottom 3 bits (EN, RXON, TXON) may be
	 * written while the block is running.  TXCLR/RXCLR (bits 3-4) require
	 * EN=0.  STBY is kept set to avoid de-asserting it and needing another
	 * 4-clock wait before the subsequent enable.
	 */

	// Clear FIFOs before enabling PCM block
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, CS_A_TXCLR | CS_A_RXCLR);

	/* DMA completion is signalled via the DMA engine, not the I2S IRQ. */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, 0);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, INTx_A_ALL);

	/* TX DREQ fires when FIFO level ≤ 16; RX DREQ fires when level ≥ 48. */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_DREQ_A,
	    DREQ_A_TX_PANIC(0x10) | DREQ_A_RX_PANIC(0x30) |
	    DREQ_A_TX(0x10)       | DREQ_A_RX(0x30));

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
	    CS_A_EN | CS_A_STBY | CS_A_DMAEN);

	return (0);
}

static int
bcm2835_i2s_dai_intr(device_t dev, struct snd_dbuf *play_buf,
    struct snd_dbuf *rec_buf)
{
	struct bcm2835_i2s_softc *sc;
	uint32_t status, inten, val;
	uint8_t *samples;
	uint32_t count, size, rp, written;
	int ret = 0;

	sc = device_get_softc(dev);
	BCM2835_I2S_LOCK(sc);

	status = BCM2835_I2S_READ_4(sc, BCM_I2S_INTSTC_A);
	inten  = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);

	/*
	 * TX: FIFO empty (TXTHR=00 default).  Each stereo frame occupies
	 * two 32-bit FIFO words: one for CH1 (left) and one for CH2 (right),
	 * each carrying the 16-bit sample right-justified.
	 */
	if ((status & inten & INTx_A_TXW) && play_buf != NULL) {
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, INTx_A_TXW);

		samples = play_buf->buf;
		size    = play_buf->bufsize;
		rp      = sndbuf_getreadyptr(play_buf);
		count   = sndbuf_getready(play_buf);
		written = 0;

		/*
		 * Fill the FIFO while it will accept writes (CS_A_TXD) and
		 * the PCM buffer has at least one complete stereo frame (4 bytes).
		 */
		while (count >= 4 &&
		    (BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A) & CS_A_TXD)) {
			/* CH1 – left sample, 16-bit LE in lower word bits */
			val = (uint32_t)samples[rp % size] |
			    ((uint32_t)samples[(rp + 1) % size] << 8);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A, val);
			/* CH2 – right sample */
			val = (uint32_t)samples[(rp + 2) % size] |
			    ((uint32_t)samples[(rp + 3) % size] << 8);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A, val);
			rp      = (rp + 4) % size;
			written += 4;
			count   -= 4;
		}

		sc->play_ptr = (sc->play_ptr + written) % size;
		ret |= AUDIO_DAI_PLAY_INTR;
	}

	/*
	 * RX: FIFO at or above the RXR threshold.  Drain in stereo-frame
	 * pairs (CH1 then CH2) into the record buffer.
	 */
	if ((status & inten & INTx_A_RXR) && rec_buf != NULL) {
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, INTx_A_RXR);

		samples = rec_buf->buf;
		size    = rec_buf->bufsize;
		rp      = sndbuf_getfreeptr(rec_buf);
		count   = sndbuf_getfree(rec_buf);
		written = 0;

		while (count >= 4 &&
		    (BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A) & CS_A_RXD)) {
			/* CH1 – left */
			val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);
			samples[rp % size]       = val & 0xff;
			samples[(rp + 1) % size] = (val >> 8) & 0xff;
			/* CH2 – right */
			val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);
			samples[(rp + 2) % size] = val & 0xff;
			samples[(rp + 3) % size] = (val >> 8) & 0xff;
			rp      = (rp + 4) % size;
			written += 4;
			count   -= 4;
		}

		sc->rec_ptr = (sc->rec_ptr + written) % size;
		ret |= AUDIO_DAI_REC_INTR;
	}

	/* Clear any error latches that fired regardless of INTEN. */
	if (status & (INTx_A_TXERR | INTx_A_RXERR))
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A,
		    status & (INTx_A_TXERR | INTx_A_RXERR));

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
			/*
			 * Clear the TX FIFO, then pre-fill it with silence.
			 * TXCLR leaves the FIFO empty so TXW asserts and
			 * latches into INTSTC_A immediately.  If we enable
			 * INTEN_A with that latch set, the hardware interrupt
			 * fires on another CPU core while chn_trigger() still
			 * holds the channel lock, causing a mutex recursion
			 * panic.  Filling the FIFO puts it above the TXW
			 * threshold so the latch can be cleared before INTEN_A
			 * is enabled, deferring the first interrupt until the
			 * silence has been consumed and the lock is released.
			 *
			 * Always OR in CS_A_EN: PCMTRIG_STOP clears EN so
			 * that the block is fully gated when idle, meaning cs
			 * read back here may not have it set on the second and
			 * subsequent plays.
			 */
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
			    cs | CS_A_EN | CS_A_TXCLR);
			for (int i = 0; i < 64; i++)
				BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A, 0);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, INTx_A_TXW);
			inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A,
			    inten | INTx_A_TXW);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
			    cs | CS_A_EN | CS_A_TXON);
		} else {
			/*
			 * Flush the RX FIFO before enabling reception.
			 * RXCLR must not be set while RXON is set (BCM2835 §8.6),
			 * so clear in a separate write.
			 */
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
			    cs | CS_A_EN | CS_A_RXCLR);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, INTx_A_RXR);
			inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A,
			    inten | INTx_A_RXR);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
			    cs | CS_A_EN | CS_A_RXON);
		}
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
		if (pcm_dir == PCMDIR_PLAY)
			inten &= ~INTx_A_TXW;
		else
			inten &= ~INTx_A_RXR;
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, inten);

		cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
		if (pcm_dir == PCMDIR_PLAY) {
			cs &= ~CS_A_TXON;
			cs |= CS_A_TXCLR;
		} else {
			cs &= ~CS_A_RXON;
			cs |= CS_A_RXCLR;
		}
		if (!(cs & (CS_A_TXON | CS_A_RXON)))
			cs &= ~CS_A_EN;
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, cs);

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
	if (pcm_dir == PCMDIR_PLAY)
		ptr = sc->play_ptr;
	else
		ptr = sc->rec_ptr;
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
	uint32_t mode;

	sc = device_get_softc(dev);
	if (sc->clkman == NULL)
		return (speed);

	mode = BCM2835_I2S_READ_4(sc, BCM_I2S_MODE_A);
	if (mode & MODE_A_CLKM)
		return (speed);

	if (bcm2835_clkman_set_frequency(sc->clkman, BCM_PCM_CLKSRC,
	    speed * BCM2835_I2S_FRAME_LEN) == 0)
		device_printf(sc->dev, "could not set PCM clock\n");

	return (speed);
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
