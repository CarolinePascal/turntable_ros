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
        echo gpibtest64 ; \
      else \
        echo gpibtest32 ; \
      fi; \
    fi
endef

LD_LIBRARY_PATH := $(shell $(ld_path)) 
EXE_NAME := $(shell $(exe_name))
                                                     
gpibtest: gpibtest.o conio.o
	gcc -o $(EXE_NAME) gpibtest.o conio.o -l$(LD_LIBRARY_PATH)

conio.o: conio.c conio.h
	gcc -c $(CFLAGS) conio.c

gpibtest.o: gpibtest.c
	gcc -c $(CFLAGS) gpibtest.c

clean:
	rm -f $(EXE_NAME) *.o *~
