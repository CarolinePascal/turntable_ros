/***************************************************************************
 ibClr.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibClr.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2001,2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
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
#include <stdlib.h>

int ibclr( int ud )
{
	uint8_t cmd[ 16 ];
	ibConf_t *conf;
	ibBoard_t *board;
	ssize_t count;
	int i, result=0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

	board = interfaceBoard( conf );

	if( conf->is_interface )
	{
		setIberr( EARG );
		return func_exit( ud, 1 );
	}
	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	i = create_sent_cmd_string( conf, cmd );
	cmd[ i++ ] = SDC;

	count = _ibcmd( conf, cmd, i );
	if(count != i)
	{
		result = func_exit( ud, 1 );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
//	        pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return result;		
	}
	result = func_exit( ud, 0 );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	return result;
}


int _DevClearList( ibConf_t *conf, const Addr4882_t addressList[] )
{
	int i;
	ibBoard_t *board;
	uint8_t *cmd;
	int count;

	if( check_addressList( addressList ) == 0 )
	{
		return -1;
	}

	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		return -1;
	}

	board = interfaceBoard( conf );

	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return -1;
	}

	cmd = malloc( 16 + 2 * numofAddresses( addressList ) );
	if( cmd == NULL )
	{
		setIberr( EDVR );
		setIbcnt( ENOMEM );
		return -1;
	}

	i = 0;
	if( numofAddresses( addressList ) )
	{
		i += board_send_setup( board, addressList, cmd );
		cmd[ i++ ] = SDC;
	}
	else
	{
		cmd[ i++ ] = DCL;
	}
	count = _ibcmd( conf, cmd, i );

	free( cmd );
	cmd = NULL;

	if(count != i)
	{
		return -1;
	}

	return 0;
}

void DevClearList( int boardID, const Addr4882_t addressList[] )
{
	int retval=0;
	ibConf_t *conf;

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	pthread_mutex_lock( m_lock[boardID] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	retval = _DevClearList( conf, addressList );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
//	        pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;		
	}

	func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
//        pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	return;

	
}

void DevClear( int boardID, Addr4882_t address )
{
	Addr4882_t addressList[2];

	addressList[0] = address;
	addressList[1] = NOADDR;

	DevClearList( boardID, addressList );
}
