.PATH: ${SRCTOP}/sys/arm/broadcom/bcm2835 \
       ${SRCTOP}/sys/dev/sound \
       ${SRCTOP}/sys/dev/sound/fdt

KMOD=	bcm2835_i2s_dma
SRCS=	bcm2835_i2s_dma.c

SRCS+=	device_if.h bus_if.h ofw_bus_if.h \
 	audio_dai_if.h \
 	opt_platform.h opt_sound.h opt_snd.h \
 	channel_if.h

.include <bsd.kmod.mk>
