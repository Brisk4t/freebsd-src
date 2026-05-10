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
 *   &i2s { interrupts = <1 23>; };   -- GPU IRQ 55 (bank 1, bit 23)
 *
 * This driver targets the raw PCM/I2S hardware registers.  The existing
 * bcm2835_audio(4) driver is a separate, unrelated path through the
 * VideoCore firmware (VCHIQ) and drives the HDMI/analogue outputs only.
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

#include <dev/ofw/openfirm.h>

#include "opt_snd.h"
#include <dev/sound/pcm/sound.h>
#include <dev/sound/fdt/audio_dai.h>
#include "audio_dai_if.h"

#include "bcm2835_i2s.h"

static struct ofw_compat_data compat_data[] = {
	{ "brcm,bcm2835-i2s",	1 },
	{ NULL,			0 }
};

/*
 * CPRMAN PCM clock registers (offsets from cprman base).
 * The password field must accompany every write or the hardware ignores it.
 */
#define CPRMAN_PCM_CTL		0x98
#define CPRMAN_PCM_DIV		0x9c
#define CPRMAN_PASSWD		0x5a000000u
#define CPRMAN_ENAB		(1u << 4)
#define CPRMAN_BUSY		(1u << 7)
#define CPRMAN_SRC_PLLD		6u		/* 500 MHz on BCM2835/2711 */
#define CPRMAN_PLLD_HZ		500000000u

static uint32_t
bcm2835_i2s_set_pcm_clock(struct bcm2835_i2s_softc *sc, uint32_t hz)
{
	uint32_t div, ctl;
	int i;

	/* Stop: clear ENAB, keep PLLD as source so the clock restarts cleanly. */
	bus_space_write_4(sc->cprman_bst, sc->cprman_bsh, CPRMAN_PCM_CTL,
	    CPRMAN_PASSWD | CPRMAN_SRC_PLLD);
	for (i = 0; i < 10; i++) {
		ctl = bus_space_read_4(sc->cprman_bst, sc->cprman_bsh,
		    CPRMAN_PCM_CTL);
		if (!(ctl & CPRMAN_BUSY))
			break;
		DELAY(1000);
	}
	if (ctl & CPRMAN_BUSY) {
		device_printf(sc->dev, "PCM clock failed to stop\n");
		return (0);
	}

	if (hz == 0)
		return (0);

	div = CPRMAN_PLLD_HZ / hz;
	if (div < 4 || div > 0xfff) {
		device_printf(sc->dev, "PCM clock %u Hz out of range\n", hz);
		return (0);
	}

	bus_space_write_4(sc->cprman_bst, sc->cprman_bsh, CPRMAN_PCM_DIV,
	    CPRMAN_PASSWD | (div << 12));
	bus_space_write_4(sc->cprman_bst, sc->cprman_bsh, CPRMAN_PCM_CTL,
	    CPRMAN_PASSWD | CPRMAN_ENAB | CPRMAN_SRC_PLLD);

	for (i = 0; i < 10; i++) {
		ctl = bus_space_read_4(sc->cprman_bst, sc->cprman_bsh,
		    CPRMAN_PCM_CTL);
		if (ctl & CPRMAN_BUSY)
			break;
		DELAY(1000);
	}
	if (!(ctl & CPRMAN_BUSY)) {
		device_printf(sc->dev, "PCM clock failed to start\n");
		return (0);
	}

	return (CPRMAN_PLLD_HZ / div);
}

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

static int bcm2835_i2s_detach(device_t dev);

static int
bcm2835_i2s_init(struct bcm2835_i2s_softc *sc)
{
	uint32_t val;

	if (sc->cprman_size != 0) {
		if (bcm2835_i2s_set_pcm_clock(sc,
		    BCM2835_I2S_SAMPLING_RATE * BCM2835_I2S_FRAME_LEN) == 0)
			device_printf(sc->dev, "cannot set pcm clock frequency\n");
	}

	/* Exit RAM standby, then clear FIFOs and interrupts. */
	val = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
	val |= CS_A_STBY;
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, val);
	DELAY(100);

	val |= CS_A_TXCLR | CS_A_RXCLR;
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, val);

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, 0);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A,
	    INTx_A_RXERR | INTx_A_TXERR | INTx_A_RXR | INTx_A_TXW);

	/*
	 * TXTHR=01: TXW fires when FIFO is at most 3/4 full (≤48 entries),
	 * guaranteeing at least 16 words of headroom on every interrupt.
	 * RXTHR=01: RXR fires when FIFO is at least 1/4 full (≥16 entries).
	 */
	val = CS_A_STBY | CS_A_TXTHR(1) | CS_A_RXTHR(1);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, val);

	return (0);
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
bcm2835_i2s_attach(device_t dev)
{
	struct bcm2835_i2s_softc *sc;
	phandle_t node;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	// Initialize the mutex before allocating resources, so that we can safely log errors.
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (bus_alloc_resources(dev, bcm2835_i2s_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources\n");
		error = ENXIO;
		goto fail;
	}

	/*
	 * Map the CPRMAN (clock manager) registers directly via the cprman
	 * phandle carried in the i2s DTS "clocks" property.  This avoids a
	 * runtime dependency on bcm2835_clkman and lets the module be
	 * loaded/unloaded independently.
	 */
	node = ofw_bus_get_node(dev);
	{
		pcell_t clk_prop[2];
		phandle_t cprman_node;

		sc->cprman_size = 0;
		if (OF_getencprop(node, "clocks", clk_prop,
		    sizeof(clk_prop)) >= (ssize_t)sizeof(clk_prop)) {
			cprman_node = OF_node_from_xref(clk_prop[0]);
			if (cprman_node == -1 ||
			    OF_decode_addr(cprman_node, 0, &sc->cprman_bst,
			    &sc->cprman_bsh, &sc->cprman_size) != 0) {
				sc->cprman_size = 0;
				device_printf(dev,
				    "cannot map cprman, continuing without clock control\n");
			}
		} else {
			device_printf(dev,
			    "no clocks property, continuing without clock control\n");
		}
	}
	
	error = bcm2835_i2s_init(sc);
	if (error != 0)
		goto fail;

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);

fail:
	bcm2835_i2s_detach(dev);
	return (error);
}

static int
bcm2835_i2s_detach(device_t dev)
{
	struct bcm2835_i2s_softc *sc;

	sc = device_get_softc(dev);

	if (sc->cprman_size != 0)
		bus_space_unmap(sc->cprman_bst, sc->cprman_bsh, sc->cprman_size);

	if (sc->intrhand != NULL)
		bus_teardown_intr(dev, sc->res[1], sc->intrhand);

	bus_release_resources(dev, bcm2835_i2s_spec, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
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
		/* BCM2835 drives BCLK and LRCLK */
		break;
	case AUDIO_DAI_CLOCK_CBS_CFS:
		/* Codec drives BCLK and LRCLK */
		mode |= MODE_A_CLKM | MODE_A_FSM;
		break;
	default:
		return (EINVAL);
	}

	/*
	 * Set frame geometry and channel positions.  All modes use 32-bit
	 * slots (BCM2835_I2S_FRAME_LEN = 64 BCLK) except DSP which packs
	 * both channels back-to-back in one half-frame.
	 */
	flen = BCM2835_I2S_FRAME_LEN;

	switch (fmt) {
	case AUDIO_DAI_FORMAT_I2S:
		mode |= MODE_A_FSI;		/* I2S: FS active-low for CH1 */
		fslen = flen / 2;
		ch1pos = 1;			/* 1-bit delay after FS edge */
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

	/* Polarity fields apply additional inversion on top of format defaults. */
	if (AUDIO_DAI_POLARITY_INVERTED_BCLK(pol))
		mode ^= MODE_A_CLKI;
	if (AUDIO_DAI_POLARITY_INVERTED_FRAME(pol))
		mode ^= MODE_A_FSI;

	mode |= MODE_A_FLEN(flen - 1) | MODE_A_FSLEN(fslen);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_MODE_A, mode);

	/* CHxWID field: actual_width - 8, so 16-bit → 8 */
	chc = CHxC_CH1EN | CHxC_CH1POS(ch1pos) |
	      CHxC_CH1WID(BCM2835_I2S_CHWIDTH - 8) |
	      CHxC_CH2EN | CHxC_CH2POS(ch2pos) |
	      CHxC_CH2WID(BCM2835_I2S_CHWIDTH - 8);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_TXC_A, chc);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_RXC_A, chc);

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

	intstc = BCM2835_I2S_READ_4(sc, BCM_I2S_INTSTC_A);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, intstc);	/* W1C */

	if (intstc & INTx_A_TXW) {
		uint8_t *samples;
		uint32_t count, size, readyptr, written;

		count    = sndbuf_getready(play_buf);
		size     = play_buf->bufsize;
		readyptr = sndbuf_getreadyptr(play_buf);
		samples  = play_buf->buf;
		written  = 0;

		/*
		 * The FIFO stores one channel sample per 32-bit word; left
		 * and right alternate.  We write complete L/R pairs to keep
		 * channel alignment stable: check TXD once per pair (before
		 * the left word), then write both words unconditionally.
		 * With TXTHR=01 there are at least 16 free slots when TXW
		 * fires, so the right word will not overflow.
		 */
		while (count >= 4) {
			cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
			if (!(cs & CS_A_TXD))
				break;

			/* left sample */
			val = (uint32_t)samples[readyptr % size] |
			      ((uint32_t)samples[(readyptr + 1) % size] << 8);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A, val);
			readyptr += 2;

			/* right sample */
			val = (uint32_t)samples[readyptr % size] |
			      ((uint32_t)samples[(readyptr + 1) % size] << 8);
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A, val);
			readyptr += 2;

			written += 4;
			count   -= 4;
		}

		sc->play_ptr += written;
		sc->play_ptr %= size;
		if (written > 0)
			ret |= AUDIO_DAI_PLAY_INTR;
	}

	if (intstc & INTx_A_RXR) {
		uint8_t *samples;
		uint32_t count, size, freeptr, recorded;

		count   = sndbuf_getfree(rec_buf);
		size    = rec_buf->bufsize;
		freeptr = sndbuf_getfreeptr(rec_buf);
		samples = rec_buf->buf;
		recorded = 0;

		while (count >= 4) {
			cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
			if (!(cs & CS_A_RXD))
				break;

			/* left sample */
			val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);
			samples[freeptr++ % size] = val & 0xff;
			samples[freeptr++ % size] = (val >> 8) & 0xff;

			/* right sample */
			val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);
			samples[freeptr++ % size] = val & 0xff;
			samples[freeptr++ % size] = (val >> 8) & 0xff;

			recorded += 4;
			count    -= 4;
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
		inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);
		if (pcm_dir == PCMDIR_PLAY)
			inten |= INTx_A_TXW;
		else
			inten |= INTx_A_RXR;
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, inten);

		cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
		if (pcm_dir == PCMDIR_PLAY)
			cs |= CS_A_EN | CS_A_TXON | CS_A_TXCLR;
		else
			cs |= CS_A_EN | CS_A_RXON | CS_A_RXCLR;
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, cs);
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
		/* Disable the interface only when both directions are idle. */
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
	return (0);
}

static int
bcm2835_i2s_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{
	struct bcm2835_i2s_softc *sc;

	sc = device_get_softc(dev);
	if (sc->cprman_size == 0)
		return (ENXIO);

	if (bcm2835_i2s_set_pcm_clock(sc, rate) == 0) {
		device_printf(sc->dev, "could not set pcm clock to %u Hz\n", rate);
		return (EIO);
	}

	return (0);
}

static uint32_t
bcm2835_i2s_dai_set_chanspeed(device_t dev, uint32_t speed)
{
	struct bcm2835_i2s_softc *sc;
	uint32_t mode;
	uint64_t bclk_freq;

	sc = device_get_softc(dev);
	if (sc->cprman_size == 0)
		return (speed);

	mode = BCM2835_I2S_READ_4(sc, BCM_I2S_MODE_A);
	if (mode & MODE_A_CLKM)		/* slave: codec owns the clock */
		return (speed);

	bclk_freq = (uint64_t)speed * BCM2835_I2S_FRAME_LEN;

	if (bcm2835_i2s_set_pcm_clock(sc, (uint32_t)bclk_freq) == 0)
		device_printf(sc->dev, "could not set pcm clock\n");

	return (speed);
}

// Map generic method calls to our implementation functions.
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

// Define the driver and attach it to the simplebus.
static driver_t bcm2835_i2s_driver = {
	"i2s",
	bcm2835_i2s_methods,
	sizeof(struct bcm2835_i2s_softc),
};

DRIVER_MODULE(bcm2835_i2s, simplebus, bcm2835_i2s_driver, 0, 0);
SIMPLEBUS_PNP_INFO(compat_data);
