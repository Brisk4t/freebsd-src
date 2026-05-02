/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024
 *
 * BCM2835 I2S/PCM audio DAI driver definitions.
 */

#ifndef _BCM2835_I2S_H_
#define _BCM2835_I2S_H_

/* Register map (base 0x7e203000, size 0x24) */

#define BCM_I2S_CS_A        0x00 /* Control and status */
#define BCM_I2S_FIFO_A      0x04 /* FIFO data */

#define BCM_I2S_MODE_A      0x08 /* Mode control */
#define BCM_I2S_RXC_A       0x0c /* RX channel control */

#define BCM_I2S_TXC_A       0x10 /* TX channel control */
#define BCM_I2S_DREQ_A      0x14 /* DMA request level */

#define BCM_I2S_INTEN_A     0x18 /* Interrupt enable */
#define BCM_I2S_INTSTC_A    0x1c /* Interrupt status and clear */
#define BCM_I2S_GRAY        0x20 /* PCM Gray mode control */

/* CS_A */
#define CS_A_STBY		(1u << 25)	/* 1 = not in standby */
#define CS_A_SYNC		(1u << 24)
#define CS_A_RXSEX		(1u << 23)
#define CS_A_RXF		(1u << 22)	/* RX FIFO full (RO) */
#define CS_A_TXE		(1u << 21)	/* TX FIFO empty (RO) */
#define CS_A_RXD		(1u << 20)	/* RX FIFO has data (RO) */
#define CS_A_TXD		(1u << 19)	/* TX FIFO has space (RO) */
#define CS_A_RXR		(1u << 18)	/* RX needs reading (RO) */
#define CS_A_TXW		(1u << 17)	/* TX needs writing (RO) */
#define CS_A_RXERR		(1u << 16)
#define CS_A_TXERR		(1u << 15)
#define CS_A_RXTHR(n)	(((n) & 3u) << 7)
#define CS_A_TXTHR(n)	(((n) & 3u) << 5)
#define CS_A_RXCLR		(1u << 4)
#define CS_A_TXCLR		(1u << 3)
#define CS_A_TXON		(1u << 2)
#define CS_A_RXON		(1u << 1)
#define CS_A_EN			(1u << 0)

/* MODE_A */
// Bits 29-31 are reserved, set to 0 as per datasheet
#define MODE_A_CLK_DIS	(1u << 28)  /* 1 = disable PCM clock */
#define MODE_A_CLKM		(1u << 23)	/* 1 = PCM_CLK slave mode */
#define MODE_A_CLKI		(1u << 22)	/* invert PCM_CLK */
#define MODE_A_FSM		(1u << 21)	/* Frame Sync mode (1 = Master, 0 = Slave) */
#define MODE_A_FSI		(1u << 20)	/* Invert Frame Sync */
#define MODE_A_FLEN(n)	(((n) & 0x3ffu) << 10)	/* frame len - 1 */
#define MODE_A_FSLEN(n)	(((n) & 0x3ffu) << 0)	/* FS assert clocks */

/* TXC_A / RXC_A — identical layout */
#define CHxC_CH1WEX		(1u << 31)              /* channel 1 width extension */
#define CHxC_CH1EN		(1u << 30)              /* 1 = enable channel 1 (left) */
#define CHxC_CH1POS(n)	(((n) & 0x3ffu) << 20)  /* channel 1 position in frame */
#define CHxC_CH1WID(n)	(((n) & 0xfu) << 16)    /* channel 1 width: actual_width - 8 */
#define CHxC_CH2WEX		(1u << 15)              /* channel 2 width extension */
#define CHxC_CH2EN		(1u << 14)              /* 1 = enable channel 2 (right) */
#define CHxC_CH2POS(n)	(((n) & 0x3ffu) << 4)   /* channel 2 position in frame */
#define CHxC_CH2WID(n)	(((n) & 0xfu) << 0)     /* channel 2 width: actual_width - 8 */

/* INTEN_A / INTSTC_A — write 1 to INTSTC_A to clear */
#define INTx_A_RXERR	(1u << 3)
#define INTx_A_TXERR	(1u << 2)
#define INTx_A_RXR		(1u << 1)
#define INTx_A_TXW		(1u << 0)

/*
 * Fixed frame geometry: 2 × 32-bit slots = 64 BCLK per stereo frame.
 * BCLK = sample_rate × 64.  Each FIFO word carries one 16-bit sample
 * (left or right), right-justified in a 32-bit word.
 */
#define BCM2835_I2S_FRAME_LEN		64
#define BCM2835_I2S_CHWIDTH		    16	/* bits per sample */
#define BCM2835_I2S_FIFO_SIZE		64	/* 32-bit words */
#define BCM2835_I2S_SAMPLING_RATE	48000

struct bcm2835_i2s_softc {
	device_t		 dev;
	bus_space_tag_t		 cprman_bst;
	bus_space_handle_t	 cprman_bsh;
	bus_size_t		 cprman_size;	/* 0 if not mapped */
	struct resource		*res[2];
	struct mtx		 mtx;
	void			*intrhand;
	uint32_t		 play_ptr;
	uint32_t		 rec_ptr;
};

#endif /* _BCM2835_I2S_H_ */
