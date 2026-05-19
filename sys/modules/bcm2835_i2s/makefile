.PATH: ${SRCTOP}/sys/arm/broadcom/bcm2835 \
       ${SRCTOP}/sys/dev/sound/fdt

KMOD=	bcm2835_i2s
SRCS=	bcm2835_i2s.c
SRCS+=	device_if.h bus_if.h ofw_bus_if.h
SRCS+=	audio_dai_if.h
SRCS+=	opt_platform.h opt_sound.h
SRCS+=	channel_if.h

 

.include <bsd.kmod.mk>
