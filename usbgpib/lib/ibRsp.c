/***************************************************************************
 ibRsp.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibRsp.c of the Linux GPIB Package driver 
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
 
#include "ib_internal.h"

extern pthread_mutex_t* m_lock[GPIB_MAX_NUM_BOARDS];

static int serial_poll( ibBoard_t *board, unsigned int pad, int sad,
	unsigned int usec_timeout, char *result )
{
	serial_poll_ioctl_t poll_cmd;
	int retval;

	poll_cmd.pad = pad;
	poll_cmd.sad = sad;

	set_timeout( board, usec_timeout );

	retval = ioctl( board->fileno, IBRSP, &poll_cmd );
	if(retval < 0)
	{
		switch( errno )
		{
			case ETIMEDOUT:
				setIberr( EABO );
				return -5;
				break;
			case EPIPE:
				setIberr( ESTB );
				break;
			default:
				setIberr( EDVR );
				setIbcnt( errno );
				break;
		}
		return -1;
	}

	*result = poll_cmd.status_byte;

	return 0;
}

int ibrsp(int ud, char *spr)
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval, result=0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[conf->settings.board] );
	board = interfaceBoard( conf );
	//yuan modify 11/21/05
	if( conf->is_interface && check_cic( board ))
	{
		setIberr( EARG );
  		result = func_exit( ud, 1 );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
	    	return result; 
	}

	if( conf->async.in_progress && (!(conf->async.ibsta & CMPL)))
	{
	    setIberr( EOIP );
	    result = _func_exit( ud, 1, 0, 0, 0, 0, 1 );
	    //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	    pthread_mutex_unlock( m_lock[conf->settings.board] );
	    return result; 
	}
	retval = serial_poll( board, conf->settings.pad, conf->settings.sad,
		conf->settings.spoll_usec_timeout, spr );
	if(retval < 0)
	{
		if( errno == ETIMEDOUT )
			conf->timed_out = 1;
		result = func_exit( ud, 1 );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;	
	}

	result = func_exit( ud, 0 );
        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}

void AllSpoll( int boardID, const Addr4882_t addressList[], short resultList[] )
{
	int i;
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;

	setIbcnt( 0 );
	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	if( check_addressList( addressList ) == 0 )
	{
		func_exit( boardID, 1 );
		return;
	}

	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		func_exit( boardID, 1 );
		return;
	}

	pthread_mutex_lock( m_lock[boardID] );
	board = interfaceBoard( conf );

	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	if( conf->async.in_progress && (!(conf->async.ibsta & CMPL)))
	{
	    setIberr( EOIP );
	    _func_exit( boardID, 1, 0, 0, 0, 0, 1 );
	    pthread_mutex_unlock( m_lock[boardID] );
	    return; 
	}
	setIbsta( 0 );
	retval = 0;
	for( i = 0; i < numofAddresses( addressList ); i++ )
	{
		char result;
		retval = serial_poll( board, _getPAD( addressList[ i ] ),
			_getSAD( addressList[ i ] ), conf->settings.spoll_usec_timeout, &result );
		if( retval < 0 )
		{
			if( errno == ETIMEDOUT )
				conf->timed_out = 1;
			break;
		}
		resultList[ i ] = result & 0xff;
	}
	setIbcnt( i );

	if( retval < 0 ) func_exit( boardID, 1 );
	else func_exit( boardID, 0 );
	pthread_mutex_unlock(m_lock[boardID] );
	return;
}

void FindRQS( int boardID, const Addr4882_t addressList[], short *result )
{
	int i;
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;

	setIbcnt( 0 );
	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	if( check_addressList( addressList ) == 0 )
	{
		func_exit( boardID, 1 );
		return;
	}

	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		func_exit( boardID, 1 );
		return;
	}

	pthread_mutex_lock( m_lock[boardID] );
	setIbsta( 0 );
	board = interfaceBoard( conf );
	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	retval = 0;
	for( i = 0; i < numofAddresses( addressList ); i++ )
	{
		char spoll_byte;
		retval = serial_poll( board, _getPAD( addressList[ i ] ),
			_getSAD( addressList[ i ] ), conf->settings.usec_timeout, &spoll_byte );
		if( retval < 0 )
		{
			if( errno == ETIMEDOUT )
				conf->timed_out = 1;
			break;
		}
		if( spoll_byte & request_service_bit )
		{
			*result = spoll_byte & 0xff;
			break;
		}
	}
	setIbcnt( i );
	if( i == numofAddresses( addressList ) )
	{
		setIberr( ETAB );
		retval = -1;
	}

	if( retval < 0 ) func_exit( boardID, 1 );
	else func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}

void ReadStatusByte( int boardID, Addr4882_t address, short *result )
{
	ibConf_t *conf;
	ibBoard_t *board;
	char byte_result;
	int retval;

	*result=0;
	setIbcnt( 0 );
	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	if( check_address( address ) == 0 )
	{
		func_exit( boardID, 1 );
		return;
	}

	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		func_exit( boardID, 1 );
		return;
	}

	pthread_mutex_lock( m_lock[boardID] );
	setIbsta( 0 );
	board = interfaceBoard( conf );

	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}

	retval = serial_poll( board, _getPAD( address ),
		_getSAD( address ), conf->settings.spoll_usec_timeout, &byte_result );
	if( retval < 0 )
	{
		if( errno == ETIMEDOUT )
			conf->timed_out = 1;
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	*result = byte_result & 0xff;
	func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}
