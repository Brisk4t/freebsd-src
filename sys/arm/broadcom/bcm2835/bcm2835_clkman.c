/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2017 Poul-Henning Kamp <phk@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
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
 *
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sema.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/broadcom/bcm2835/bcm2835_clkman.h>

/*
 * Crystal oscillator rates: BCM2835 uses a 19.2 MHz XOSC; BCM2711 uses 54 MHz.
 * All PLL frequencies are derived from these references by the firmware.
 */
#define BCM2835_XOSC_RATE	19200000u
#define BCM2711_XOSC_RATE	54000000u

/*
 * A2W PLLD register offsets within the CPRMAN address space.
 * Both BCM2835 and BCM2711 share the same 0x2000-byte CPRMAN window.
 */
#define A2W_PLLD_CTRL		0x1140
#define A2W_PLLD_FRAC		0x1240
#define A2W_PLLD_ANA1		0x1054	/* ANA register bank (base 0x1050) + 4 */
#define A2W_PLLD_PER		0x1540	/* PLLD "per" output channel divider */

#define A2W_PLL_CTRL_NDIV_MASK	0x000003ffu
#define A2W_PLL_CTRL_PDIV_MASK	0x00007000u
#define A2W_PLL_CTRL_PDIV_SHIFT	12
#define A2W_PLL_FRAC_MASK	0x000fffffu
#define A2W_PLL_FRAC_BITS	20

/* A2W_PLLD_PER field masks */
#define A2W_PLLD_PER_DISABLE	(1u << 8)	/* channel gated off */
#define A2W_PLLD_PER_DIV_MASK	0x000000ffu	/* 8-bit integer divider */

/*
 * On BCM2835, ANA1 bit 14 enables a feedback pre-divider that doubles the
 * effective NDIV and FDIV.  BCM2711 repurposes this bit for VCO_RANGE, so
 * it must not be treated as a pre-divider on that SoC.
 */
#define A2W_PLLD_ANA1_FB_PREDIV	(1u << 14)

struct bcm2835_clkman_soc_data {
	uint32_t	xosc_rate;
	bool		has_prediv;
};

static const struct bcm2835_clkman_soc_data bcm2711_soc_data = {
	.xosc_rate = BCM2711_XOSC_RATE,
	.has_prediv = false,
};

static const struct bcm2835_clkman_soc_data bcm2835_soc_data = {
	.xosc_rate = BCM2835_XOSC_RATE,
	.has_prediv = true,
};

static struct ofw_compat_data compat_data[] = {
	{"brcm,bcm2711-cprman",		(uintptr_t)&bcm2711_soc_data},
	{"brcm,bcm2835-cprman",		(uintptr_t)&bcm2835_soc_data},
	{"broadcom,bcm2835-cprman",	(uintptr_t)&bcm2835_soc_data},
	{NULL,				0}
};

struct bcm2835_clkman_softc {
	device_t		sc_dev;

	struct resource *	sc_m_res;
	bus_space_tag_t		sc_m_bst;
	bus_space_handle_t	sc_m_bsh;

	uint32_t		sc_plld_freq;
	uint32_t		sc_xosc_rate;
	bool			sc_has_prediv;
};

#define BCM_CLKMAN_WRITE(_sc, _off, _val)              \
    bus_space_write_4(_sc->sc_m_bst, _sc->sc_m_bsh, _off, _val)
#define BCM_CLKMAN_READ(_sc, _off)                     \
    bus_space_read_4(_sc->sc_m_bst, _sc->sc_m_bsh, _off)

#define W_CMCLK(_sc, unit, _val) BCM_CLKMAN_WRITE(_sc, unit, 0x5a000000 | (_val))
#define R_CMCLK(_sc, unit) BCM_CLKMAN_READ(_sc, unit)
#define W_CMDIV(_sc, unit, _val) BCM_CLKMAN_WRITE(_sc, (unit) + 4, 0x5a000000 | (_val))
#define R_CMDIV(_sc,  unit) BCM_CLKMAN_READ(_sc, (unit) + 4)

/*
 * Read the PLLD_per frequency from A2W hardware registers.
 *
 * The CPRMAN CM clock source SRC=6 is "PLLD per", which is the PLLD VCO
 * divided by the PER output channel divider (A2W_PLLD_PER).  Two stages:
 *
 *   VCO  = xosc * (ndiv * 2^FRAC_BITS + fdiv) / (pdiv * 2^FRAC_BITS)
 *   freq = VCO / per_div
 *
 * On BCM2835 the firmware leaves per_div=1 so PLLD_per == VCO (~500 MHz).
 * On BCM2711 the firmware sets VCO ~3 GHz and per_div=4 giving ~750 MHz.
 *
 * Returns 0 if the PLL is unconfigured (pdiv == 0) or the PER channel is
 * gated off.
 */
static uint32_t
bcm2835_clkman_plld_freq(struct bcm2835_clkman_softc *sc)
{
	uint32_t ctrl, frac, ndiv, pdiv, per_reg, per_div;
	uint64_t rate;

	ctrl = BCM_CLKMAN_READ(sc, A2W_PLLD_CTRL);
	frac = BCM_CLKMAN_READ(sc, A2W_PLLD_FRAC) & A2W_PLL_FRAC_MASK;
	ndiv = ctrl & A2W_PLL_CTRL_NDIV_MASK;
	pdiv = (ctrl & A2W_PLL_CTRL_PDIV_MASK) >> A2W_PLL_CTRL_PDIV_SHIFT;

	if (pdiv == 0)
		return (0);

	/*
	 * On BCM2835, check the feedback pre-divider.  When active it doubles
	 * the effective integer and fractional divisors.  BCM2711 does not have
	 * this feature (the bit is reused for VCO_RANGE), so skip the check.
	 */
	if (sc->sc_has_prediv &&
	    (BCM_CLKMAN_READ(sc, A2W_PLLD_ANA1) & A2W_PLLD_ANA1_FB_PREDIV)) {
		ndiv *= 2;
		frac *= 2;
	}

	rate = (uint64_t)sc->sc_xosc_rate *
	    (((uint64_t)ndiv << A2W_PLL_FRAC_BITS) + frac);
	rate /= pdiv;
	rate >>= A2W_PLL_FRAC_BITS;

	/* Apply the PLLD_per output channel divider. */
	per_reg = BCM_CLKMAN_READ(sc, A2W_PLLD_PER);
	if (per_reg & A2W_PLLD_PER_DISABLE)
		return (0);
	per_div = per_reg & A2W_PLLD_PER_DIV_MASK;
	if (per_div > 1)
		rate /= per_div;

	return ((uint32_t)rate);
}

static int
bcm2835_clkman_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "BCM283x Clock Manager");

	return (BUS_PROBE_DEFAULT);
}

static int
bcm2835_clkman_attach(device_t dev)
{
	struct bcm2835_clkman_softc *sc;
	const struct bcm2835_clkman_soc_data *soc;
	int rid;

	if (device_get_unit(dev) != 0) {
		device_printf(dev, "only one clk manager supported\n");
		return (ENXIO);
	}

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	soc = (const struct bcm2835_clkman_soc_data *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	sc->sc_xosc_rate = soc->xosc_rate;
	sc->sc_has_prediv = soc->has_prediv;

	rid = 0;
	sc->sc_m_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (!sc->sc_m_res) {
		device_printf(dev, "cannot allocate memory window\n");
		return (ENXIO);
	}

	sc->sc_m_bst = rman_get_bustag(sc->sc_m_res);
	sc->sc_m_bsh = rman_get_bushandle(sc->sc_m_res);

	sc->sc_plld_freq = bcm2835_clkman_plld_freq(sc);
	if (sc->sc_plld_freq == 0) {
		device_printf(dev, "PLLD not configured by firmware\n");
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_m_res);
		return (ENXIO);
	}

	device_printf(dev, "PLLD_per frequency: %u Hz (xosc %u Hz, per_div %u)\n",
	    sc->sc_plld_freq, sc->sc_xosc_rate,
	    BCM_CLKMAN_READ(sc, A2W_PLLD_PER) & A2W_PLLD_PER_DIV_MASK);

	SYSCTL_ADD_UINT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "plld_freq", CTLFLAG_RD, &sc->sc_plld_freq, 0,
	    "PLLD VCO frequency in Hz (read from hardware at attach)");

	bus_attach_children(dev);
	return (0);
}

uint32_t
bcm2835_clkman_set_frequency(device_t dev, uint32_t unit, uint32_t hz)
{
	struct bcm2835_clkman_softc *sc;
	int i;
	uint32_t u;

	sc = device_get_softc(dev);

	if (unit != BCM_PWM_CLKSRC && unit != BCM_PCM_CLKSRC) {
		device_printf(sc->sc_dev,
		    "Unsupported unit 0x%x", unit);
		return (0);
	}

	W_CMCLK(sc, unit, 6);
	for (i = 0; i < 10; i++) {
		u = R_CMCLK(sc, unit);
		if (!(u&0x80))
			break;
		DELAY(1000);
	}
	if (u & 0x80) {
		device_printf(sc->sc_dev,
		    "Failed to stop clock for unit 0x%x", unit);
		return (0);
	}
	if (hz == 0)
		return (0);

	u = sc->sc_plld_freq / hz;
	if (u < 4) {
		device_printf(sc->sc_dev,
		    "Frequency too high for unit 0x%x (max: %u MHz)",
		    unit, sc->sc_plld_freq / 4 / 1000000);
		return (0);
	}
	if (u > 0xfff) {
		device_printf(sc->sc_dev,
		    "Frequency too low for unit 0x%x (min: %u kHz)",
		    unit, sc->sc_plld_freq / 0xfff / 1000);
		return (0);
	}
	hz = sc->sc_plld_freq / u;
	W_CMDIV(sc, unit, u << 12);

	W_CMCLK(sc, unit, 0x16);
	for (i = 0; i < 10; i++) {
		u = R_CMCLK(sc, unit);
		if ((u&0x80))
			break;
		DELAY(1000);
	}
	if (!(u & 0x80)) {
		device_printf(sc->sc_dev,
		    "Failed to start clock for unit 0x%x", unit);
		return (0);
	}
	return (hz);
}

static int
bcm2835_clkman_detach(device_t dev)
{
	struct bcm2835_clkman_softc *sc;

	bus_generic_detach(dev);

	sc = device_get_softc(dev);
	if (sc->sc_m_res)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_m_res);

	return (0);
}

static device_method_t bcm2835_clkman_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		bcm2835_clkman_probe),
	DEVMETHOD(device_attach,	bcm2835_clkman_attach),
	DEVMETHOD(device_detach,	bcm2835_clkman_detach),

	DEVMETHOD_END
};

static driver_t bcm2835_clkman_driver = {
	"bcm2835_clkman",
	bcm2835_clkman_methods,
	sizeof(struct bcm2835_clkman_softc),
};

DRIVER_MODULE(bcm2835_clkman, simplebus, bcm2835_clkman_driver, 0, 0);
MODULE_VERSION(bcm2835_clkman, 1);
