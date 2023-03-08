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
        echo master_cmds64 ; \
      else \
        echo master_cmds32 ; \
      fi; \
    fi
endef

LD_LIBRARY_PATH := $(shell $(ld_path)) 
EXE_NAME := $(shell $(exe_name))
                                                     
master_cmds: master_cmds.o conio.o
	gcc -o $(EXE_NAME) conio.o master_cmds.o -l$(LD_LIBRARY_PATH)

conio.o: conio.c
	gcc -c $(CFLAGS) conio.c

master_cmds.o: master_cmds.c
	gcc -c $(CFLAGS) master_cmds.c

clean:
	rm -f $(EXE_NAME) *.o *~
