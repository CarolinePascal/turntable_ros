# Makefile for a usbgpibfl kernel module
 
CC=gcc

ifneq ($(KERNELRELEASE),)
obj-m	+= usbgpibfl.o
usbgpibfl-objs := usbgpib_fwdl.o fileread.o usbgpib_ext.o usbgpib_int.o

else
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)

default:	
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	mkdir -p ../../drivers/$(shell uname -r)
	mv usbgpibfl.ko ../../drivers/$(shell uname -r)
	strip --strip-debug --strip-unneeded ../../drivers/$(shell uname -r)/usbgpibfl.ko
endif

install:
	cp ../../drivers/$(shell uname -r)/usbgpibfl.ko /lib/modules/$(shell uname -r)/kernel/drivers/usb
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

