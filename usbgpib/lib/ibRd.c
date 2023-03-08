/***************************************************************************
 ibRd.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibRd.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2001,2002,2003 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/
 
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 	
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include "ib_internal.h"

//extern pthread_mutex_t* m_lock[GPIB_MAX_NUM_BOARDS];
int _ReceiveSetup( ibConf_t *conf, Addr4882_t address )
{
	ibBoard_t *board;
	uint8_t cmdString[8];
	unsigned int i = 0;
	unsigned int pad, board_pad;
	int sad, board_sad;

	if( check_address( address ) == 0 ||
		address == NOADDR )
	{
		setIberr( EARG );
		return -1;
	}
	board = interfaceBoard( conf );

	if( ask_pad( board, &board_pad ) < 0 ) return -1;
	if( ask_sad( board, &board_sad ) < 0 ) return -1;

	pad = _getPAD( address );
	sad = _getSAD( address );

	cmdString[ i++ ] = UNL;

	cmdString[ i++ ] = MLA( board_pad );	/* controller's listen address */
	if ( board_sad >= 0 )
		cmdString[ i++ ] = MSA( board_sad );
	cmdString[ i++ ] = MTA( pad );
	if( sad >= 0 )
		cmdString[ i++ ] = MSA( sad );
	if(conf->settings.readdr) {
		//yuan add 02/29/08
		if ( _ibcmd_rw( conf, cmdString, i, 1 ) < 0)
		//if ( _ibcmd( conf, cmdString, i ) < 0)
		{
				return -1;
		}
		//printf("%s \n","my_ibcmd"); 

	} else {
		if ( _ibcmd_TA( conf, cmdString, i, address) < 0)	
		{
				return -1;
		}
			//printf("%s \n","my_ibcmd_TA"); 
	}
	return 0;
}

ssize_t read_data(ibConf_t *conf, uint8_t *buffer, size_t count, size_t *bytes_read,  uint8_t *end)
{
	ibBoard_t *board;
	read_write_ioctl_t read_cmd;
	int retval;

	board = interfaceBoard( conf );
        *end = 0;
	read_cmd.buffer = buffer;
	read_cmd.count = count;
	read_cmd.handle = conf->handle;
	read_cmd.end = 0;

	if(conf->settings.usec_timeout)
		read_cmd.usec_timeout = (conf->settings.usec_timeout>=LINETIMEOUT)? (conf->settings.usec_timeout+ADDTIMEOUT):conf->settings.usec_timeout;//conf->settings.usec_timeout;
	else
		read_cmd.usec_timeout = 0;
	
	conf->end = 0;
	board->io_in_progress = 1;
	retval = ioctl( board->fileno, IBRD, &read_cmd );
	if( retval < 0 )
	{
		switch( errno )
		{
			case ETIMEDOUT:
				conf->timed_out = 1;
				setIberr(EABO);
				break;
			default:
			   if(conf->async.in_progress)
				conf->async.iberr = EDVR;
			   else
				setIberr(EDVR);
				setIbcnt(errno);
				break;
		}
		board->io_in_progress = 0;
		return -1;
	}
	board->io_in_progress = 0;
        if(read_cmd.handle <0){
		if(read_cmd.handle == -ETIMEDOUT) {
			conf->timed_out = 1; 
			setIberr( EABO );
		} else {			
		   if(conf->async.in_progress)
				conf->async.iberr = EDVR;
		   else
				setIberr( EDVR );
		}

	}
	if( read_cmd.end ) {
		conf->end = 1;
		*end = 1;
		/////conf->end_timeout = read_cmd.usec_timeout;
	//printf("end: %x\n", conf->end);
	} else {
		*end = 0;	
	}
	////if(conf->end != read_cmd.end)
	/////printf("end1: %x\n", conf->end);
	//yuan add 03/02/06
	*bytes_read = read_cmd.count;
//printf("end2: %x\n", conf->end);
	if(read_cmd.handle<0) {
	   return read_cmd.handle;
	} else {
	   return read_cmd.count;
	}

//	if( read_cmd.end ) conf->end = 1;

//	*bytes_read = read_cmd.count;

//	return retval;
}

ssize_t _ibrd( ibConf_t *conf, uint8_t *buffer, size_t count, size_t *bytes_read, uint8_t* end)
{
	*end = 0;
	if(conf->async.in_progress) {
		conf->async.opstart = 1;
		//SetEvent( conf->async.startE);
	}
	if((interfaceBoard( conf )->is_system_controller) || ( conf->is_interface == 0)) {
		//yuan modify 03/24/06
		//printf("%s \n","iblcleos");
	    iblcleos( conf );
	}
	*bytes_read = 0;
	// set eos mode
	//iblcleos( conf );

	if( conf->is_interface == 0 )
	{
		if(conf->async.in_progress && conf->async.abort) {
			conf->async.abort = 0;
			return -1;
		}
		// addressing
		if( _ReceiveSetup( conf, _mkAddr( conf->settings.pad, conf->settings.sad ) ) < 0 )
		{
			return -1;
		}
	}
	if(conf->async.in_progress && conf->async.abort) {
		    conf->async.abort = 0;
			return -1;
	}
	return read_data( conf, buffer, count, bytes_read, end);
	//return read_data(conf, buffer, count, bytes_read);
}

int ibrd(int ud, void *rd, size_t cnt)
{
	ibConf_t *conf;
	ssize_t retval;
	size_t bytes_read;
        uint8_t end;
        int result=0;
//struct sembuf   semBuf;
	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[conf->settings.board] );
/*{

      semBuf.sem_num = 0;
       semBuf.sem_op = -1;
       semBuf.sem_flg = SEM_UNDO;
       if (semop(shr_sem[0], &semBuf, 1) != 0)
                printf(" RC_LOCK_ERROR\n");
}*/

	if( conf->async.in_progress && (!(conf->async.ibsta & CMPL)))
	{
		setIberr( EOIP );
		result = _func_exit( ud, 1, 0, 0, 0, 0, 1 );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}
	conf->async.in_progress = 0;
	setIbcnt( 0 );
	setIbsta( 0 );
	retval = _ibrd( conf, rd, cnt, &bytes_read, &end);
//	retval = my_ibrd(conf, rd, cnt, &bytes_read);
	if(retval < 0)
	{
		if(ThreadIberr() != EDVR)
			setIbcnt(bytes_read);
		result = func_exit( ud, 1 );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}else
	{
		setIbcnt(bytes_read);
   	//setIbcnt( count );
//yuan add 02/21/08
	conf->end = end;
	}

	result = _func_exit( ud, 0, 0, 0, DCAS, 0, 0 );
	//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
/*semBuf.sem_num = 0;
       semBuf.sem_op  = 1;
       semBuf.sem_flg = SEM_UNDO;

       if (semop(shr_sem[0], &semBuf, 1) != 0)
           printf(" RC_UNLOCK_ERROR\n");*/
	return result;
}

int ibrda( int ud, void *buffer, size_t cnt )
{
	ibConf_t *conf;
	int retval;
	int result = 0;
	conf = _func_init( ud, 1, 0 );
	if( conf == NULL )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );
	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
        setIbcnt( 0 );
	setIbsta( 0 );	
	retval = aio_setup( ud, conf, GPIB_AIO_READ,
		buffer, cnt );
	if( retval < 0 ) 
	{
		result = _func_exit( ud, 1, 0, 0, 0, 0, 1 );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}

	result = _func_exit( ud, 0, 0, 0, CMPL, 0, 1 );
	//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
//printf("leave ibrda\n");
	return result;
}

int ibrdf(int ud, const char *file_path )
{
	ibConf_t *conf;
	int retval;
	uint8_t buffer[ 0x4000 ];
	unsigned long byte_count;
	FILE *save_file;
	int error;
 uint8_t end;
int result=0;
 
	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

	save_file = fopen( file_path, "a" );
	if( save_file == NULL )
	{
		setIberr( EFSO );
		setIbcnt( errno );
		return func_exit( ud, 1 );
	}

	if( conf->is_interface == 0 )
	{
		// set up addressing
		if( _ReceiveSetup( conf, _mkAddr( conf->settings.pad, conf->settings.sad ) ) < 0 )
		{
			return func_exit( ud, 1 );
		}
	}

	// set eos mode
	//iblcleos( conf );
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[conf->settings.board] );
	byte_count = error = 0;
	do
	{
		int fwrite_count;
		size_t bytes_read;

		retval = read_data(conf, buffer, sizeof(buffer), &bytes_read, &end);
		fwrite_count = fwrite( buffer, 1, bytes_read, save_file );
		if( fwrite_count != bytes_read )
		{
			setIberr( EFSO );
			setIbcnt( errno );
			error++;
		}
		byte_count += fwrite_count;
		if( retval < 0 )
		{
			error++;
			break;
		}
	}while( conf->end == 0 && error == 0 );

	setIbcnt( byte_count );

	if( fclose( save_file ) )
	{
		setIberr( EFSO );
		setIbcnt( errno );
		result = func_exit( ud, 1 );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}
	if( error ) {
	 result = func_exit( ud, error );
	 //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	 pthread_mutex_unlock( m_lock[conf->settings.board] );
	 return result;
	}

	result = _func_exit( ud, 0, 0, 0, DCAS, 0, 0 );
	//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}

int _RcvRespMsg( ibConf_t *conf, void *buffer, size_t count, int termination )
{
	ibBoard_t *board;
	int retval;
	int use_eos;
	size_t bytes_read;
	 uint8_t end;

	if( conf->is_interface == 0 )
	{
		setIberr(EARG);
		return -1;
	}

	board = interfaceBoard( conf );
//yuan remove 01/12/12
/**	
	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return -1;
	}
	//YUAN ADD 10/05/06
	if(board->is_system_controller) {
	 if( check_cic( board ) == 0 )
	 {
		setIberr( ECIC );
		return -1;
	 }
	}
**/
	if( termination != ( termination & 0xff ) &&
		termination != STOPend )
	{
		setIberr( EARG );
		return -1;
	}
	use_eos = ( termination != STOPend );
	retval = config_read_eos( board, use_eos, termination, 1 );
	if( retval < 0 )
	{
		return retval;
	}

	retval = read_data(conf, buffer, count, &bytes_read, &end);
	setIbcnt(bytes_read);
	if(retval < 0)
	{
		return -1;
	}
        setIbcnt(retval);

	return 0;
}

void RcvRespMsg( int boardID, void *buffer, size_t count, int termination )
{
	ibConf_t *conf;
	int retval;

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[boardID] );
	conf->async.in_progress = 0;
	setIbcnt( 0 );	
	setIbsta( 0 );
	retval = _RcvRespMsg( conf, buffer, count, termination );
	if( retval < 0 )
	{		
		func_exit( boardID, 1 );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );		
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	_func_exit( boardID, 0, 0, 0, DCAS, 0, 0 );
	//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[boardID] );
}

void ReceiveSetup( int boardID, Addr4882_t address )
{
	ibConf_t *conf;
	int retval;

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	pthread_mutex_lock( m_lock[boardID] );
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	retval = _ReceiveSetup( conf, address );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
//		pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return;
	}

	func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
	//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	return;
}

int _Receive( ibConf_t *conf, Addr4882_t address,
	void *buffer, size_t count, int termination )
{
	int retval;

	retval = _ReceiveSetup( conf, address );
	if( retval < 0 ) return retval;

	retval = _RcvRespMsg( conf, buffer, count, termination );
	if( retval < 0 )return retval;

	return 0;
}

void Receive( int boardID, Addr4882_t address,
	void *buffer, size_t count, int termination )
{
	ibConf_t *conf;
	int retval;

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[boardID] );
	conf->async.in_progress = 0;
        setIbcnt( 0 );
	setIbsta( 0 );
	retval = _Receive( conf, address, buffer, count, termination );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
//		pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	_func_exit( boardID, 0, 0, 0, DCAS, 0, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}
