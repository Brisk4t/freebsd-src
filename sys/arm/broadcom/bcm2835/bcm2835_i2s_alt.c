/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024
 *
 * BCM2835 I2S/PCM audio DAI driver (Interrupt-driven/PIO alternative).
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

#define BCM2835_I2S_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define BCM2835_I2S_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define BCM2835_I2S_READ_4(sc, reg)	bus_read_4((sc)->res[0], (reg))
#define BCM2835_I2S_WRITE_4(sc, reg, val) bus_write_4((sc)->res[0], (reg), (val))

static uint32_t sc_fmt[] = {
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	0
};

static struct pcmchan_caps bcm2835_i2s_caps = {
	BCM2835_I2S_SAMPLING_RATE, BCM2835_I2S_SAMPLING_RATE, sc_fmt, 0
};

static inline void
bcm2835_pcm_wait(struct bcm2835_i2s_softc *sc, uint32_t cycles)
{
	uint32_t iter = (cycles + 1) / 2;
	uint32_t val;
	uint32_t sync_bit = 0;

	val = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
	if (val & CS_A_SYNC)
		sync_bit = CS_A_SYNC;

	while (iter-- > 0) {
		sync_bit ^= CS_A_SYNC;
		val = (val & ~CS_A_SYNC) | sync_bit;
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, val);

		while ((BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A) & CS_A_SYNC) != sync_bit)
			; /* spin */
	}
}

/* 
 * audio_dai handlers 
 */
static int
bcm2835_i2s_dai_init(device_t dev, uint32_t format)
{
	struct bcm2835_i2s_softc *sc = device_get_softc(dev);
	uint32_t val;
	uint32_t fmt = AUDIO_DAI_FORMAT_FORMAT(format);
	uint32_t pol = AUDIO_DAI_FORMAT_POLARITY(format);
	uint32_t clk = AUDIO_DAI_FORMAT_CLOCK(format);

	BCM2835_I2S_LOCK(sc);

	val = BCM2835_I2S_READ_4(sc, BCM_I2S_MODE_A);
	val &= ~(MODE_A_FSI | MODE_A_CLKI | MODE_A_FSM | MODE_A_CLKM | MODE_A_FLEN(0x3ff) | MODE_A_FSLEN(0x3ff));

	/* Frame length is fixed 64 (2x32-bit slots), FS asserts for 32 clocks (half frame) */
	val |= MODE_A_FLEN(BCM2835_I2S_FRAME_LEN - 1);
	val |= MODE_A_FSLEN((BCM2835_I2S_FRAME_LEN / 2) - 1);

	/* Clock Master/Slave */
	switch (clk) {
	case AUDIO_DAI_CLOCK_CBM_CFM:
		/* Codec is master, BCM2835 is slave for BCLK and LRCLK */
		val |= MODE_A_CLKM | MODE_A_FSM;
		break;
	case AUDIO_DAI_CLOCK_CBS_CFS:
		/* BCM2835 is master, Codec is slave */
		break;
	default:
		BCM2835_I2S_UNLOCK(sc);
		return (EINVAL);
	}

	/* Format (I2S vs Left-Justified) */
	uint32_t txc = BCM2835_I2S_READ_4(sc, BCM_I2S_TXC_A) & ~(CHxC_CH1POS(0x3ff) | CHxC_CH2POS(0x3ff) | CHxC_CH1WID(0xf) | CHxC_CH2WID(0xf));
	uint32_t rxc = BCM2835_I2S_READ_4(sc, BCM_I2S_RXC_A) & ~(CHxC_CH1POS(0x3ff) | CHxC_CH2POS(0x3ff) | CHxC_CH1WID(0xf) | CHxC_CH2WID(0xf));

	/* Always 16-bit widths for now */
	txc |= CHxC_CH1WID(BCM2835_I2S_CHWIDTH - 8) | CHxC_CH2WID(BCM2835_I2S_CHWIDTH - 8);
	rxc |= CHxC_CH1WID(BCM2835_I2S_CHWIDTH - 8) | CHxC_CH2WID(BCM2835_I2S_CHWIDTH - 8);
	txc |= CHxC_CH1EN | CHxC_CH2EN;
	rxc |= CHxC_CH1EN | CHxC_CH2EN;

	switch (fmt) {
	case AUDIO_DAI_FORMAT_I2S:
		/* Channel 1 starts at clock 1, Channel 2 at clock 33 */
		txc |= CHxC_CH1POS(1) | CHxC_CH2POS(33);
		rxc |= CHxC_CH1POS(1) | CHxC_CH2POS(33);
		break;
	case AUDIO_DAI_FORMAT_LJ:
		/* Channel 1 starts at clock 0, Channel 2 at clock 32 */
		txc |= CHxC_CH1POS(0) | CHxC_CH2POS(32);
		rxc |= CHxC_CH1POS(0) | CHxC_CH2POS(32);
		break;
	default:
		BCM2835_I2S_UNLOCK(sc);
		return (EINVAL);
	}

	/* Polarity (Invert BCLK or Frame Sync) */
	switch (pol) {
	case AUDIO_DAI_POLARITY_IB_IF:
		val |= MODE_A_CLKI | MODE_A_FSI;
		break;
	case AUDIO_DAI_POLARITY_IB_NF:
		val |= MODE_A_CLKI;
		break;
	case AUDIO_DAI_POLARITY_NB_IF:
		val |= MODE_A_FSI;
		break;
	case AUDIO_DAI_POLARITY_NB_NF:
		/* Normal */
		break;
	}

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_TXC_A, txc);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_RXC_A, rxc);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_MODE_A, val);

	BCM2835_I2S_UNLOCK(sc);

	return (0);
}

static int
bcm2835_i2s_dai_set_sysclk(device_t dev, unsigned int rate, int dai_dir)
{
	/* Handled in set_chanspeed for dynamic rate */
	return (0);
}

static uint32_t
bcm2835_i2s_dai_set_chanspeed(device_t dev, uint32_t speed)
{
	struct bcm2835_i2s_softc *sc = device_get_softc(dev);

	if (bcm2835_clkman_set_frequency(sc->clkman, BCM_PCM_CLKSRC, speed * BCM2835_I2S_FRAME_LEN) != 0) {
		device_printf(dev, "failed to set PCM clock speed\n");
		return (0);
	}
	sc->hw_started = true;
	return (speed);
}

static uint32_t
bcm2835_i2s_dai_set_chanformat(device_t dev, uint32_t format)
{
	return (0);
}

static int
bcm2835_i2s_dai_setup_intr(device_t dev, driver_intr_t intr_handler, void *intr_arg)
{
	struct bcm2835_i2s_softc *sc;

	sc = device_get_softc(dev);

	if (bus_setup_intr(dev, sc->res[1], INTR_TYPE_AV | INTR_MPSAFE, 
	    NULL, intr_handler, intr_arg, &sc->intrhand)) {
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	return (0);
}

static int
bcm2835_i2s_dai_intr(device_t dev, struct snd_dbuf *play_buf, struct snd_dbuf *rec_buf)
{
	struct bcm2835_i2s_softc *sc = device_get_softc(dev);
	uint32_t status;
	int ret = 0;

	BCM2835_I2S_LOCK(sc);

	status = BCM2835_I2S_READ_4(sc, BCM_I2S_INTSTC_A);
	if (status == 0) {
		BCM2835_I2S_UNLOCK(sc);
		return (0);
	}

	/* Clear the interrupts that fired W1C */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A, status);

	/* Playback (TX) */
	if ((status & INTx_A_TXW) && play_buf != NULL) {
		uint8_t *samples = play_buf->buf;
		uint32_t size = play_buf->bufsize;
		uint32_t count = sndbuf_getready(play_buf);
		uint32_t readyptr = sndbuf_getreadyptr(play_buf);
		uint32_t written = 0;

		/* Write whilst FIFO has space and we have samples to provide */
		while ((BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A) & CS_A_TXD) && count >= 2) {
			/* 
			 * BCM2835 I2S wants 1 16-bit sample per 32-bit FIFO word.
			 * S16_LE in FreeBSD buffer is byte-interleaved: [L L] [R R]
			 */
			uint32_t val = samples[readyptr % size] | 
				      (samples[(readyptr + 1) % size] << 8);
			
			BCM2835_I2S_WRITE_4(sc, BCM_I2S_FIFO_A, val);

			readyptr += 2;
			written += 2;
			count -= 2;
		}

		sc->play_ptr += written;
		sc->play_ptr %= size;
		if (written > 0)
			ret |= AUDIO_DAI_PLAY_INTR;
	}

	/* Record (RX) */
	if ((status & INTx_A_RXR) && rec_buf != NULL) {
		uint8_t *samples = rec_buf->buf;
		uint32_t size = rec_buf->bufsize;
		uint32_t count = sndbuf_getfree(rec_buf);
		uint32_t freeptr = sndbuf_getfreeptr(rec_buf);
		uint32_t recorded = 0;

		/* Read whilst FIFO has data and we have space internally */
		while ((BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A) & CS_A_RXD) && count >= 2) {
			uint32_t val = BCM2835_I2S_READ_4(sc, BCM_I2S_FIFO_A);

			samples[freeptr % size] = val & 0xff;
			samples[(freeptr + 1) % size] = (val >> 8) & 0xff;

			freeptr += 2;
			recorded += 2;
			count -= 2;
		}

		sc->rec_ptr += recorded;
		sc->rec_ptr %= size;
		if (recorded > 0)
			ret |= AUDIO_DAI_REC_INTR;
	}

	BCM2835_I2S_UNLOCK(sc);

	return (ret);
}

static int
bcm2835_i2s_dai_trigger(device_t dev, int go, int pcmdir)
{
	struct bcm2835_i2s_softc *sc = device_get_softc(dev);
	uint32_t cs, inten;

	BCM2835_I2S_LOCK(sc);

	cs = BCM2835_I2S_READ_4(sc, BCM_I2S_CS_A);
	inten = BCM2835_I2S_READ_4(sc, BCM_I2S_INTEN_A);

	switch (go) {
	case PCMTRIG_START:
		if (pcmdir == PCMDIR_PLAY) {
			sc->play_ptr = 0;
			inten |= (INTx_A_TXW | INTx_A_TXERR);
			cs |= CS_A_TXON;
		} else {
			sc->rec_ptr = 0;
			inten |= (INTx_A_RXR | INTx_A_RXERR);
			cs |= CS_A_RXON;
		}
		break;

	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		if (pcmdir == PCMDIR_PLAY) {
			inten &= ~(INTx_A_TXW | INTx_A_TXERR);
			cs &= ~CS_A_TXON;
			cs |= CS_A_TXCLR; /* clear tx fifo */
		} else {
			inten &= ~(INTx_A_RXR | INTx_A_RXERR);
			cs &= ~CS_A_RXON;
			cs |= CS_A_RXCLR; /* clear rx fifo */
		}
		break;
	}

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, inten);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, cs);

	if (go == PCMTRIG_STOP || go == PCMTRIG_ABORT)
		bcm2835_pcm_wait(sc, 2); /* Wait 2 cycles for clears to take effect */

	BCM2835_I2S_UNLOCK(sc);

	return (0);
}

static uint32_t
bcm2835_i2s_dai_get_ptr(device_t dev, int pcm_dir)
{
	struct bcm2835_i2s_softc *sc = device_get_softc(dev);
	uint32_t ptr;

	BCM2835_I2S_LOCK(sc);
	if (pcm_dir == PCMDIR_PLAY)
		ptr = sc->play_ptr;
	else
		ptr = sc->rec_ptr;
	BCM2835_I2S_UNLOCK(sc);

	return ptr;
}

static struct pcmchan_caps *
bcm2835_i2s_dai_get_caps(device_t dev)
{
	return (&bcm2835_i2s_caps);
}

static int
bcm2835_i2s_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

		
	device_set_desc(dev, "BCM2835 I2S (PIO/Interrupt-Driven)");
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

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (bus_alloc_resources(dev, bcm2835_i2s_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources\n");
		error = ENXIO;
		goto fail;
	}

	sc->clkman = devclass_get_device(devclass_find("bcm2835_clkman"), 0);
	if (sc->clkman == NULL) {
		device_printf(dev, "cannot find bcm2835_clkman\n");
		error = ENXIO;
		goto fail;
	}

	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTEN_A, 0);
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_INTSTC_A,
	    INTx_A_RXERR | INTx_A_TXERR | INTx_A_RXR | INTx_A_TXW);

	/* Configure PCM while disabled. */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
	    CS_A_STBY | CS_A_TXTHR(1) | CS_A_RXTHR(1));

	/* Enable the PCM block */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
	    CS_A_EN | CS_A_STBY | CS_A_TXTHR(1) | CS_A_RXTHR(1));
	bcm2835_pcm_wait(sc, 4);

	/* Clear FIFOs */
	BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A,
	    CS_A_EN | CS_A_STBY | CS_A_TXTHR(1) | CS_A_RXTHR(1) | CS_A_TXCLR | CS_A_RXCLR);
	bcm2835_pcm_wait(sc, 2);

	node = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(node), dev);

	return (0);

fail:
	return (error);
}

static int
bcm2835_i2s_detach(device_t dev)
{
	struct bcm2835_i2s_softc *sc = device_get_softc(dev);

	if (sc->intrhand != NULL)
		bus_teardown_intr(sc->dev, sc->res[1], sc->intrhand);

	if (sc->hw_started) {
		BCM2835_I2S_WRITE_4(sc, BCM_I2S_CS_A, 0);
		sc->hw_started = false;
	}

	bus_release_resources(dev, bcm2835_i2s_spec, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

static device_method_t bcm2835_i2s_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcm2835_i2s_probe),
	DEVMETHOD(device_attach,	bcm2835_i2s_attach),
	DEVMETHOD(device_detach,	bcm2835_i2s_detach),

	/* audio_dai interface */
	DEVMETHOD(audio_dai_init,		bcm2835_i2s_dai_init),
	DEVMETHOD(audio_dai_set_sysclk, bcm2835_i2s_dai_set_sysclk),
	DEVMETHOD(audio_dai_set_chanspeed, bcm2835_i2s_dai_set_chanspeed),
	DEVMETHOD(audio_dai_set_chanformat, bcm2835_i2s_dai_set_chanformat),
	DEVMETHOD(audio_dai_get_caps,	bcm2835_i2s_dai_get_caps),
	DEVMETHOD(audio_dai_setup_intr, bcm2835_i2s_dai_setup_intr),
	DEVMETHOD(audio_dai_intr,		bcm2835_i2s_dai_intr),
	DEVMETHOD(audio_dai_trigger,	bcm2835_i2s_dai_trigger),
	DEVMETHOD(audio_dai_get_ptr,	bcm2835_i2s_dai_get_ptr),

	DEVMETHOD_END
};

static driver_t bcm2835_i2s_driver = {
	"bcm2835_i2s",
	bcm2835_i2s_methods,
	sizeof(struct bcm2835_i2s_softc),
};


DRIVER_MODULE(bcm2835_i2s, simplebus, bcm2835_i2s_driver, 0, 0);
MODULE_DEPEND(bcm2835_i2s, bcm2835_clkman, 1, 1, 1);
