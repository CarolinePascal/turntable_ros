/***************************************************************************
 gpib_proto.h
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from include/gpib_proto.h of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/
 
#ifndef GPIB_PROTO
#define GPIB_PROTO

#include <linux/fs.h>
#include "../include/Adldev.h"

int serial_poll_all( PU348A_DEVEXT board, unsigned int usec_timeout );
void init_gpib_descriptor( gpib_descriptor_t *desc );
int dvrsp(PU348A_DEVEXT board, unsigned int pad, int sad,
	unsigned int usec_timeout, uint8_t *result );
int ibcac(PU348A_DEVEXT board, int sync);
int ibcmd( PU348A_DEVEXT board, gpib_descriptor_t * desc, uint8_t *buf, size_t length );
int ibgts(PU348A_DEVEXT board);
int ibonline( PU348A_DEVEXT board );
int iboffline( PU348A_DEVEXT board );
short iblines( const PU348A_DEVEXT board );
int ibrd(PU348A_DEVEXT board, gpib_descriptor_t * desc, uint8_t *buf, size_t length, int *end_flag);
int ibrpp( PU348A_DEVEXT board, uint8_t *buf );
int ibrsv(PU348A_DEVEXT board, uint8_t poll_status);
void ibrsc( PU348A_DEVEXT board, int request_control );
int ibsic( PU348A_DEVEXT board, unsigned int usec_duration );
int ibsre(PU348A_DEVEXT board, int enable);
int ibpad( PU348A_DEVEXT board, unsigned int addr );
int ibsad( PU348A_DEVEXT board, int addr );
int ibeos( PU348A_DEVEXT board, int eos, int eosflags );
int ibwait(PU348A_DEVEXT board, int wait_mask, int clear_mask, int set_mask,
int *status, unsigned long usec_timeout, gpib_descriptor_t *desc );
int ibwrt(PU348A_DEVEXT board, gpib_descriptor_t * desc, uint8_t *buf, size_t cnt, int send_eoi );
int ibstatus( PU348A_DEVEXT board );
int inquire_ibstatus( PU348A_DEVEXT board, const gpib_status_queue_t *device,
	int clear_mask, int set_mask, gpib_descriptor_t *desc );
int ibppc( PU348A_DEVEXT board, uint8_t configuration );
int io_timed_out( PU348A_DEVEXT board );
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
extern void timeout_routine( unsigned long arg );
#else
extern void timeout_routine( struct timer_list *t );
#endif 
void wdt_start( PU348A_DEVEXT board, unsigned int usec_timeout );
void wdt_remove( PU348A_DEVEXT board );
int GetGPIBResponse(U348A_DEVEXT *priv, uint16_t len, uint8_t *buf);
#endif /* GPIB_PROTO */
