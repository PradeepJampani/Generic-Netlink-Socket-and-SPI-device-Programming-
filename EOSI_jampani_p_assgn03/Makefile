CC = i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = i586-poky-linux-
SDKTARGETSYSROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
export PATH:=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin:/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$(PATH)
CCFLAGS-y  += -Wall

LDLIBS = -L$(SDKTARGETSYSROOT)/usr/lib
CCFLAGS = -I$(SDKTARGETSYSROOT)/usr/include/libnl3

APP = led_sock

obj-m:= Socket_NLP.o


all:
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SDKTARGETSYSROOT)/usr/src/kernel M=$(shell pwd) modules
	$(CC) -Wall -pthread -o $(APP) Socket_NLP_user.c --sysroot=$(SDKTARGETSYSROOT)  $(CCFLAGS) -lnl-genl-3 -lnl-3


clean:
	rm -f *.ko
	rm -f *.o
	rm -f Module.symvers
	rm -f modules.order
	rm -f *.mod.c
	rm -rf .tmp_versions
	rm -f *.mod.c
	rm -f *.mod.o
	rm -f \.*.cmd
	rm -f Module.markers
	rm -f $(APP)

