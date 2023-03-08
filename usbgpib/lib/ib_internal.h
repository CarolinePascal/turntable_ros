/***************************************************************************
 ib_internal.h
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ib_internal.h of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2001, 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/
 
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _IB__H
#define _IB__H

#include "ib.h"
#include "ibP.h"
#include "gpib_ioctl.h"
#include <unistd.h>
#include <sys/ioctl.h>
#include <assert.h>

#include "ibConf.h"

enum _gpib_addr
{
	SAD_DISABLED = -1,
	ADDR_INVALID = -2
};

int _ibwait( ibConf_t *conf, int wait_mask, int clear_mask, int set_mask, int *status );
int _ibwait_a( ibConf_t *conf, int wait_mask, int clear_mask, int set_mask, int *status );
void init_async_op( struct async_operation *async );
void init_asyno_op( struct notify_operation *async );
int ibBoardOpen( ibBoard_t *board );
int ibBoardClose( ibBoard_t *board );
int iblcleos( const ibConf_t *conf );
int ibGetDescriptor(ibConf_t conf);
int ibGetIndexFromName( const char *name );
ssize_t _ibcmd( ibConf_t *conf, const uint8_t *buffer, size_t length);
ssize_t _ibcmd_TA( ibConf_t *conf, const uint8_t *buffer, size_t length, Addr4882_t address);
ssize_t _ibcmd_rw( ibConf_t *conf, const uint8_t *buffer, size_t count, short rw );
ssize_t _ibrd( ibConf_t *conf, uint8_t *buffer, size_t count, size_t *bytes_read, uint8_t* end);
int _ibwrt( ibConf_t *conf, const uint8_t *buffer, size_t count, size_t *bytes_written);
unsigned int create_sent_cmd_string( const ibConf_t *conf, uint8_t *cmdString );
unsigned int board_send_setup( const ibBoard_t *board,
	const Addr4882_t addressList[], uint8_t *cmdString );
int send_setup( ibConf_t *conf );
void init_ibconf( ibConf_t *conf );
void init_ibboard( ibBoard_t *board );
int _ibdev( ibConf_t new_conf );
int _ibbna( ibConf_t *conf, unsigned int new_board_index );
unsigned int timeout_to_usec( enum gpib_timeout timeout );
unsigned int ppoll_timeout_to_usec( unsigned int timeout );
unsigned int usec_to_ppoll_timeout( unsigned int usec );
int set_timeout( const ibBoard_t *board, unsigned int usec_timeout );
int close_device_handle( ibConf_t *conf );
int open_device_handle( ibConf_t *conf );
int change_gpib_address( ibConf_t *conf, unsigned int pad, int sad, int force );
int lock_board_mutex( ibBoard_t *board );
int unlock_board_mutex( ibBoard_t *board );
int func_exit( int ud, int error );
int _func_exit( int ud, int error, int no_sync_globals, int no_update_ibsta,
	int status_clear_mask, int status_set_mask, int no_unlock_board );
ibConf_t * func_init( int ud );
ibConf_t * _func_init( int ud, int no_lock_board, int ignore_eoip );
void setIbsta( int status );
void setIberr( int error );
void setIbcnt( int count );
int ibstatus( ibConf_t *conf, int error, int clear_mask, int set_mask );
int set_ibLon( ibBoard_t *board, signed int code);
unsigned int usec_to_timeout( unsigned int usec );
int ask_ppc( const ibBoard_t *board );
int ask_ist( const ibBoard_t *board );
int ask_pad( const ibBoard_t *board, unsigned int *pad );
int ask_sad( const ibBoard_t *board, int *sad );
int ask_board_t1_delay( const ibBoard_t *board );
int ask_board_rsv( const ibBoard_t *board );
int ask_autopoll( const ibBoard_t *board );
int conf_online( ibConf_t *conf, int online );
int configure_autospoll( ibConf_t *conf, int enable );
int _getPAD( Addr4882_t address );
int _getSAD( Addr4882_t address );
Addr4882_t _mkAddr( unsigned int pad, int sad );
int check_address( Addr4882_t address );
int check_addressList( const Addr4882_t addressList[] );
unsigned int numofAddresses( const Addr4882_t addressList[] );
int remote_enable( const ibBoard_t *board, int enable );
int config_read_eos( ibBoard_t *board, int use_eos_char,
	int eos_char, int compare_8_bits );
void sync_globals( void );
int check_sc( const ibBoard_t *board );
int check_cic( const ibBoard_t *board );
int assert_ifc( ibBoard_t *board, unsigned int usec );
int request_system_control( ibBoard_t *board, int request_control );
int set_t1_delay( ibBoard_t *board, int delay );
int _ibpad( ibConf_t *conf, unsigned int address );
int _ibsad( ibConf_t *conf, int address );
int _ibtmo( ibConf_t *conf, int timeout );
void _ibeot( ibConf_t *conf, int send_eoi );
int _ibist( ibConf_t *conf, int ist );
int _ibppc( ibConf_t *conf, int v );
int _ibsre( ibConf_t *conf, int v );
int _ibrsv( ibConf_t *conf, int v );
int _iblines( ibConf_t *conf, short *line_status );
int _ibgts( ibConf_t *conf, int shadow_handshake );
int _ibrsc( ibConf_t *conf, int request_control );
int _ibsic( ibConf_t *conf );
int _ibstop( ibConf_t *conf );
int _ibcac( ibConf_t *conf, int synchronous );
int _DevClearList( ibConf_t *conf, const Addr4882_t addressList[] );
int _ReceiveSetup( ibConf_t *conf, Addr4882_t address );
int _SendSetup( ibConf_t *conf, const Addr4882_t addressList[] );
int _SendList( ibConf_t *conf, const Addr4882_t addressList[],
	const void *buffer, size_t count, int eotmode );
int _EnableRemote( ibConf_t *conf, const Addr4882_t addressList[] );
int _Receive( ibConf_t *conf, Addr4882_t address,
	void *buffer, size_t count, int termination );

static __inline__ ibBoard_t* interfaceBoard( const ibConf_t *conf )
{
	assert( conf->settings.board >= 0 && conf->settings.board < GPIB_MAX_NUM_BOARDS );
	return &ibBoard[ conf->settings.board ];
}

#include <stdio.h>
	
enum gpib_aio_varieties
{
	GPIB_AIO_COMMAND,
	GPIB_AIO_READ,
	GPIB_AIO_WRITE,
};
int aio_setup( int ud, ibConf_t *conf, int gpib_aio_type,
	void *buffer, int cnt );
int aio_thread_join( struct async_operation *async );
int abort_aio( ibConf_t *conf);

extern pthread_mutex_t* m_lock[GPIB_MAX_NUM_BOARDS];

#include "sem_h.h"

#endif	/* _IB__H */
