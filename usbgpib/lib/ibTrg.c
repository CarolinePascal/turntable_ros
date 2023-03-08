/***************************************************************************
 ibTrg.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibTrg.c of the Linux GPIB Package driver 
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

int _trig( ibConf_t *conf, const Addr4882_t addressList[] )
{
	int i, retval;
	uint8_t *cmd;

	if( check_addressList( addressList ) == 0 )
	{
		setIberr( EARG );
		return -1;
	}

	cmd = malloc( 16 + 2 * numofAddresses( addressList ) );
	if( cmd == NULL )
	{
		setIberr( EDVR );
		setIbcnt( ENOMEM );
		return -1;
	}

	i = board_send_setup( interfaceBoard( conf ), addressList, cmd );
	cmd[ i++ ] = GET;

	retval = _ibcmd( conf, cmd, i );

	free( cmd );
	cmd = NULL;

	if( retval != i )
	{
		return -1;
	}

	return 0;
}

int ibtrg( int ud )
{
	ibConf_t *conf;
	int retval, result=0;
	Addr4882_t addressList[ 2 ];

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

	if( conf->is_interface )
	{
		setIberr( EARG );
		return func_exit( ud, 1 );
	}
	pthread_mutex_lock( m_lock[conf->settings.board] );
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	addressList[ 0 ] = _mkAddr( conf->settings.pad, conf->settings.sad );
	addressList[ 1 ] = NOADDR;

	retval = _trig( conf, addressList );
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

void TriggerList( int boardID, const Addr4882_t addressList[] )
{
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
//	pthread_mutex_lock( &semap[boardID]->lock );	
	pthread_mutex_lock( m_lock[boardID] );
	retval = _trig( conf, addressList );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
		//pthread_mutex_unlock( &semap[boardID]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}

	func_exit( boardID, 0 );
//	pthread_mutex_unlock( &semap[boardID]->lock );
	pthread_mutex_unlock( m_lock[boardID] );
}

void Trigger( int boardID, Addr4882_t address )
{
	Addr4882_t addressList[ 2 ];

	addressList[ 0 ] = address;
	addressList[ 1 ] = NOADDR;

	TriggerList( boardID, addressList );
}
