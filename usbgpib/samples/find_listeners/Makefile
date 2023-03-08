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
        echo find_ltn64 ; \
      else \
        echo find_ltn32 ; \
      fi; \
    fi
endef

LD_LIBRARY_PATH := $(shell $(ld_path)) 
EXE_NAME := $(shell $(exe_name))
                                                     
find_ltn: find_ltn.o conio.o
	gcc -o $(EXE_NAME) find_ltn.o conio.o -l$(LD_LIBRARY_PATH)

conio.o: conio.c conio.h
	gcc -c $(CFLAGS) conio.c

find_ltn.o: find_ltn.c
	gcc -c $(CFLAGS) find_ltn.c

clean:
	rm -f $(EXE_NAME) *.o *~
