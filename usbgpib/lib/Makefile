CC=gcc
CFLAGS=-O2 -Wall -Wstrict-prototypes -I../include -fPIC

define lib_name
    if [ `/bin/uname -s` = 'Linux' ] ; then \
      if [ `/bin/uname -m` = 'x86_64' ] ; then \
        echo libadgpib64.so ; \
      elif [ `/bin/uname -m` = 'aarch64' ] ; then\
      	echo libadgpib64.so ;\
      else
        echo libadgpib32.so ; \
      fi; \
    fi
endef

LIB_NAME := $(shell $(lib_name))

all: $(LIB_NAME)

.c.o:	$.c
	gcc $(CFLAGS) -c $<

$(LIB_NAME): ibCac.o ibClr.o ibCmd.o ibEos.o ibEot.o \
	ibFind.o ibLines.o ibOnl.o ibPad.o ibRd.o ibRpp.o ibRsp.o ibRsv.o \
	ibSad.o ibSic.o ibSre.o ibTmo.o ibTrg.o ibWait.o ibWrt.o \
	ibGts.o ibBoard.o ibutil.o globals.o ibask.o ibppc.o \
	ibLoc.o ibDma.o ibdev.o ibbna.o async.o ibconfig.o ibFindLstn.o \
	local_lockout.o self_test.o pass_control.o ibstop.o ibnotify.o sem_c.o -lpthread
	
	gcc -shared -lm -o $(LIB_NAME) \
	ibCac.o ibClr.o ibCmd.o ibEos.o ibEot.o \
	ibFind.o ibLines.o ibOnl.o ibPad.o ibRd.o ibRpp.o ibRsp.o ibRsv.o \
	ibSad.o ibSic.o ibSre.o ibTmo.o ibTrg.o ibWait.o ibWrt.o \
	ibGts.o ibBoard.o ibutil.o globals.o ibask.o ibppc.o \
	ibLoc.o ibDma.o ibdev.o ibbna.o async.o ibconfig.o ibFindLstn.o \
	local_lockout.o self_test.o pass_control.o ibstop.o ibnotify.o sem_c.o -lpthread

install:
	cp -f $(LIB_NAME) /usr/lib

clean:
	rm -f *.o $(LIB_NAME) *~
