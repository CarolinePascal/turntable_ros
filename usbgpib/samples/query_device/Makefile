# Makefile for sample programs

CC=gcc
CFLAGS := -Wall -Wstrict-prototypes -O2 -I../../include

define ld_path
    if [ `/bin/uname -s` = 'Linux' ] ; then \
      if [ `/bin/uname -m` = 'x86_64' ] ; then \
        echo adgpib64 ; \
      else \
        echo adgpib32 ; \
      fi; \
    fi
endef

define exe_name
    if [ `/bin/uname -s` = 'Linux' ] ; then \
      if [ `/bin/uname -m` = 'x86_64' ] ; then \
        echo query_device64 ; \
      else \
        echo query_device32 ; \
      fi; \
    fi
endef

LD_LIBRARY_PATH := $(shell $(ld_path)) 
EXE_NAME := $(shell $(exe_name))
                                                     
query_device: query_device.o conio.o
	gcc -o $(EXE_NAME) conio.o query_device.o -l$(LD_LIBRARY_PATH)

conio.o: conio.c conio.h
	gcc -c $(CFLAGS) conio.c

query_device.o: query_device.c
	gcc -c $(CFLAGS) query_device.c

clean:
	rm -f $(EXE_NAME) *.o *~
