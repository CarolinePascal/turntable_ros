# Makefile for a u348a kernel module
 
CC=gcc

ifneq ($(KERNELRELEASE),)
obj-m	+= u348a.o
u348a-objs := usb348a.o ioctl.o ibgts.o ibcac.o util.o iblines.o ibrpp.o ibrsv.o ibsic.o ibsre.o ibcmd.o ibwrite.o ibread.o device.o autopoll.o wdtimer.o

else
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)

default:	
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	mkdir -p ../../drivers/$(shell uname -r)
	mv u348a.ko ../../drivers/$(shell uname -r)
	strip --strip-debug --strip-unneeded ../../drivers/$(shell uname -r)/u348a.ko
endif

install:
	cp ../../drivers/$(shell uname -r)/u348a.ko /lib/modules/$(shell uname -r)/kernel/drivers/usb
	/sbin/depmod -a
	if [ -d "/etc/sysconfig/modules" ]; then \
	cp ../../drivers/adgpib.modules  /etc/sysconfig/modules; \
	fi
	if [ -d "/etc/modules-load.d" ]; then \
	cp ../../drivers/adgpib.conf  /etc/modules-load.d; \
	fi

clean :
	rm -f *.mod.c *.o .*.cmd .tmp_versions/*
	rmdir .tmp_versions

