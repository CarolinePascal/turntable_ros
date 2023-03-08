/***************************************************************************
 ibRpp.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibRpp.c of the Linux GPIB Package driver 
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

int _ibrpp( ibConf_t *conf, char *result )
{
	//uint8_t poll_byte;
	ppoll_ioctl_t poll_iot;
	ibBoard_t *board;
	int retval;
	
	board = interfaceBoard( conf );

	if( check_cic( board ) == 0 )
	{
		assert_ifc( board, 100 );
		board->assert_ifc = 1;
//		setIberr( ECIC );
//		return -1;
	}

	//set_timeout( board, conf->settings.ppoll_usec_timeout );
	poll_iot.ppoll_usec_timeout = (conf->settings.ppoll_usec_timeout>=LINETIMEOUT)? (conf->settings.ppoll_usec_timeout+ADDTIMEOUT):conf->settings.ppoll_usec_timeout;
	poll_iot.poll_byte = 0;
	retval = ioctl( board->fileno, IBRPP, &poll_iot );
	if( retval < 0 )
	{
		switch( errno )
		{
			case ETIMEDOUT:
				conf->timed_out = 1;
				break;
			default:
				setIberr( EDVR );
				setIbcnt( errno );
				break;
		}
		return -1;
	}

	*result =poll_iot. poll_byte;

	return 0;
}

int ibrpp( int ud, char *ppr )
{
	ibConf_t *conf;
	int retval, result=0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	retval = _ibrpp( conf, ppr );
	if( retval < 0 )
	{
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

void PPoll( int boardID, short *result )
{
	char byte_result;
	ibConf_t *conf;
	int retval;

	conf = func_init( boardID );
	if( conf == NULL )
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
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[boardID] );
	retval = _ibrpp( conf, &byte_result );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return;
	}

	*result = byte_result & 0xff;
	func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	return;
}

