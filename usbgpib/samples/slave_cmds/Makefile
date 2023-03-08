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
        echo slave_cmds64 ; \
      else \
        echo slave_cmds32 ; \
      fi; \
    fi
endef

LD_LIBRARY_PATH := $(shell $(ld_path)) 
EXE_NAME := $(shell $(exe_name))
                                                     
slave_cmds: slave_cmds.o conio.o
	gcc -o $(EXE_NAME) conio.o slave_cmds.o -l$(LD_LIBRARY_PATH)

conio.o: conio.c
	gcc -c $(CFLAGS) conio.c

slave_cmds.o: slave_cmds.c
	gcc -c $(CFLAGS) slave_cmds.c

clean:
	rm -f $(EXE_NAME) *.o *~
