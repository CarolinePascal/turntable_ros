/***************************************************************************
 local_lockout.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/local_lockout.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
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

static int local_lockout( ibConf_t *conf, const Addr4882_t addressList[] )
{
	uint8_t cmd;
	int retval;

	retval = _EnableRemote( conf, addressList );
	if( retval < 0 ) return retval;

	cmd = LLO;
	retval = _ibcmd( conf, &cmd, 1 );
	if( retval < 0 ) return retval;

	return 0;
}

void SendLLO( int boardID )
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
//	pthread_mutex_lock( &semap[boardID]->lock );
	retval = local_lockout( conf, NULL );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
//		pthread_mutex_unlock( &semap[boardID]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	func_exit( boardID, 0 );
	//pthread_mutex_unlock( &semap[boardID]->lock );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}

void SetRWLS( int boardID, const Addr4882_t addressList[] )
{
	ibConf_t *conf;
	int retval;

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}

	if( numofAddresses( addressList ) == 0 )
	{
		setIberr( EARG );
		func_exit( boardID, 1 );
		return;
	}
	pthread_mutex_lock( m_lock[boardID] );
//	pthread_mutex_lock( &semap[boardID]->lock );
	retval = local_lockout( conf, addressList );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
		//pthread_mutex_unlock( &semap[boardID]->lock );
		return;
	}
	func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
//	pthread_mutex_unlock( &semap[boardID]->lock );
	return;
}
