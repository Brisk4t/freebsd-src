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

#define BCM_I2S_CS_A	 0x00 /* Control and status */
#define BCM_I2S_FIFO_A	 0x04 /* FIFO data */
#define BCM_I2S_CS_A	 0x00 /* Control and status */
#define BCM_I2S_FIFO_A	 0x04 /* FIFO data */

#define BCM_I2S_MODE_A	 0x08 /* Mode control */
#define BCM_I2S_RXC_A	 0x0c /* RX channel control */
#define BCM_I2S_MODE_A	 0x08 /* Mode control */
#define BCM_I2S_RXC_A	 0x0c /* RX channel control */

#define BCM_I2S_TXC_A	 0x10 /* TX channel control */
#define BCM_I2S_DREQ_A	 0x14 /* DMA request level */
#define BCM_I2S_TXC_A	 0x10 /* TX channel control */
#define BCM_I2S_DREQ_A	 0x14 /* DMA request level */

#define BCM_I2S_INTEN_A	 0x18 /* Interrupt enable */
#define BCM_I2S_INTSTC_A 0x1c /* Interrupt status and clear */
#define BCM_I2S_GRAY	 0x20 /* PCM Gray mode control */
#define BCM_I2S_INTEN_A	 0x18 /* Interrupt enable */
#define BCM_I2S_INTSTC_A 0x1c /* Interrupt status and clear */
#define BCM_I2S_GRAY	 0x20 /* PCM Gray mode control */

/* CS_A */
#define CS_A_STBY   (1u << 25) /* RAM Standby Flag */
#define CS_A_SYNC   (1u << 24) /* Clock sync helper, echos after 2 PCM clocks */
#define CS_A_RXSEX  (1u << 23) /* Sign extend RX samples to 32 bits (RW) */
#define CS_A_RXF    (1u << 22) /* RX FIFO full (RO) */
#define CS_A_TXE    (1u << 21) /* TX FIFO empty (RO) */
#define CS_A_RXD    (1u << 20) /* RX FIFO has data (RO) */
#define CS_A_TXD    (1u << 19) /* TX FIFO has space (RO) */
#define CS_A_RXR    (1u << 18) /* RX needs reading (RO) */
#define CS_A_TXW    (1u << 17) /* TX needs writing (RO) */
#define CS_A_RXERR  (1u << 16) /* RX error (RO) */
#define CS_A_TXERR  (1u << 15) /* TX error (RO) */
#define CS_A_RXSYNC (1u << 14) /* RX FIFO Sync (0 = Out of Sync) */
#define CS_A_TXSYNC (1u << 13) /* TX FIFO Sync (0 = Out of Sync) */
#define CS_A_DMAEN  (1u << 9)  /* DMA DREQ Enable */
#define CS_A_RXTHR(n) (((n) & 3u) << 7) /* RX FIFO threshold for RXR (RW) */
#define CS_A_TXTHR(n) (((n) & 3u) << 5) /* TX FIFO threshold for TXW (RW) */

/* Shared threshold field values for CS_A_TXTHR(n) and CS_A_RXTHR(n) */
#define CS_A_THR_EMPTY 0u /* TX: empty;      RX: at least one sample */
#define CS_A_THR_QTR   1u /* TX: < ¼ full;   RX: ¼ full */
#define CS_A_THR_3QTR  2u /* TX: < ¾ full;   RX: ¾ full */
#define CS_A_THR_FULL  3u /* TX: full minus one sample; RX: full */

#define CS_A_RXCLR     (1u << 4) /* Clear RX FIFO (WO, self-cleared) */
#define CS_A_TXCLR     (1u << 3) /* Clear TX FIFO (WO, self-cleared) */
#define CS_A_TXON      (1u << 2) /* Enable transmission (RW) */
#define CS_A_RXON      (1u << 1) /* Enable reception (RW) */
#define CS_A_EN	       (1u << 0) /* Enable PCM Audio Interface (RW) */

/* MODE_A */
#define MODE_A_CLK_DIS (1u << 28) /* 1 = disable PCM clock */
#define MODE_A_FRXP \
	(1u << 25) /* RX packed mode: both channels in one FIFO word */
#define MODE_A_FTXP \
	(1u << 24) /* TX packed mode: one FIFO word fills both channels */
#define MODE_A_CLKM	(1u << 23) /* 1 = PCM_CLK slave (input) */
#define MODE_A_CLKI	(1u << 22) /* invert PCM_CLK */
#define MODE_A_FSM	(1u << 21) /* 1 = frame sync slave (input) */
#define MODE_A_FSI	(1u << 20) /* invert frame sync */
#define MODE_A_FLEN(n)	(((n) & 0x3ffu) << 10) /* frame len - 1 */
#define MODE_A_FSLEN(n) (((n) & 0x3ffu) << 0)  /* Frame sync length */
#define MODE_A_CLK_DIS	(1u << 28)	       /* 1 = disable PCM clock */
#define MODE_A_FRXP \
	(1u << 25) /* RX packed mode: both channels in one FIFO word */
#define MODE_A_FTXP \
	(1u << 24) /* TX packed mode: one FIFO word fills both channels */
#define MODE_A_CLKM	(1u << 23) /* 1 = PCM_CLK slave (input) */
#define MODE_A_CLKI	(1u << 22) /* invert PCM_CLK */
#define MODE_A_FSM	(1u << 21) /* 1 = frame sync slave (input) */
#define MODE_A_FSI	(1u << 20) /* invert frame sync */
#define MODE_A_FLEN(n)	(((n) & 0x3ffu) << 10) /* frame len - 1 */
#define MODE_A_FSLEN(n) (((n) & 0x3ffu) << 0)  /* Frame sync length */

/* TXC_A / RXC_A — identical layout */
#define CHxC_CH1WEX    (1u << 31)
#define CHxC_CH1EN     (1u << 30)
#define CHxC_CH1POS(n) (((n) & 0x3ffu) << 20)
#define CHxC_CH1WID(n) (((n) & 0xfu) << 16) /* actual_width - 8 */
#define CHxC_CH2WEX    (1u << 15)
#define CHxC_CH2EN     (1u << 14)
#define CHxC_CH2POS(n) (((n) & 0x3ffu) << 4)
#define CHxC_CH2WID(n) (((n) & 0xfu) << 0) /* actual_width - 8 */
#define CHxC_CH1WEX    (1u << 31)
#define CHxC_CH1EN     (1u << 30)
#define CHxC_CH1POS(n) (((n) & 0x3ffu) << 20)
#define CHxC_CH1WID(n) (((n) & 0xfu) << 16) /* actual_width - 8 */
#define CHxC_CH2WEX    (1u << 15)
#define CHxC_CH2EN     (1u << 14)
#define CHxC_CH2POS(n) (((n) & 0x3ffu) << 4)
#define CHxC_CH2WID(n) (((n) & 0xfu) << 0) /* actual_width - 8 */

/* INTEN_A / INTSTC_A — write 1 to INTSTC_A to clear */
#define INTx_A_RXERR (1u << 3) /* RX Fifo error occured */
#define INTx_A_TXERR (1u << 2) /* TX Fifo error occured */
#define INTx_A_RXR   (1u << 1) /* RX Read interrupt occured */
#define INTx_A_TXW   (1u << 0) /* TX Write interrupt occured */
#define INTx_A_ALL   (INTx_A_RXERR | INTx_A_TXERR | INTx_A_RXR | INTx_A_TXW)

/* BCM2835 PCM/I2S peripheral DREQ assignments (ARM Peripherals Table 4-3) */
#define BCM2835_I2S_DREQ_TX 2
#define BCM2835_I2S_DREQ_RX 3
#define BCM2835_I2S_DREQ_TX 2
#define BCM2835_I2S_DREQ_RX 3

/*
 * Default frame geometry: 2 × ch_width BCLK per stereo frame (tight packing).
 * BCLK = sample_rate × frame_len.  frame_len and ch_width are updated by
 * set_chanformat when the PCM format changes.
 */
#define BCM2835_I2S_FRAME_LEN	 32 /* default: 2 × 16-bit slots */
#define BCM2835_I2S_CHWIDTH	 16 /* default bits per sample */
#define BCM2835_I2S_FIFO_SIZE	 64 /* 32-bit words */
#define BCM2835_I2S_RATE_MIN	 8000
#define BCM2835_I2S_RATE_DEFAULT 48000
#define BCM2835_I2S_RATE_MAX	 192000 /* BCM2711 sustains 192 kHz */
#define BCM2835_I2S_FRAME_LEN	 32	/* default: 2 × 16-bit slots */
#define BCM2835_I2S_CHWIDTH	 16	/* default bits per sample */
#define BCM2835_I2S_FIFO_SIZE	 64	/* 32-bit words */
#define BCM2835_I2S_RATE_MIN	 8000
#define BCM2835_I2S_RATE_DEFAULT 48000
#define BCM2835_I2S_RATE_MAX	 192000 /* BCM2711 sustains 192 kHz */

struct bcm2835_i2s_softc {
	device_t dev;
	device_t clkman; /* bcm2835_clkman device */
	struct resource *res[2];
	struct mtx mtx;
	void *intrhand;
	uint32_t play_ptr;
	uint32_t rec_ptr;
	uint32_t sample_rate;	   /* current rate set by set_chanspeed */
	uint32_t txthr;		   /* CS_A_TXTHR value for current rate */
	uint32_t rxthr;		   /* CS_A_RXTHR value for current rate */
	uint32_t frame_len;	   /* BCLK per stereo frame (2 × ch_width) */
	uint32_t ch_width;	   /* hardware bits per sample */
	uint32_t bytes_per_sample; /* PCM buffer bytes per sample */
	uint32_t num_channels;	   /* 1 = mono, 2 = stereo */
	int dai_fmt; /* DAI format from dai_init (AUDIO_DAI_FORMAT_*) */

	/* true when FTXP/FRXP: one FIFO word per stereo frame */
	bool packed_mode;
};

#endif /* _BCM2835_I2S_H_ */
