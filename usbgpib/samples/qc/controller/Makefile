# Makefile for sample programs

CC=gcc
CFLAGS := -Wall -Wstrict-prototypes -O2 -I../../../include

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
        echo controller64 ; \
      else \
        echo controller32 ; \
      fi; \
    fi
endef

LD_LIBRARY_PATH := $(shell $(ld_path)) 
EXE_NAME := $(shell $(exe_name))
                                                     
controller: controller.o conio.o
	gcc -o $(EXE_NAME) controller.o conio.o -l$(LD_LIBRARY_PATH)

conio.o: conio.c conio.h
	gcc -c $(CFLAGS) conio.c

controller.o: controller.c
	gcc -c $(CFLAGS) controller.c

clean:
	rm -f $(EXE_NAME) *.o *~
