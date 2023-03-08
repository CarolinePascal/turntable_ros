/***************************************************************************
 ibSre.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibSre.c of the Linux GPIB Package driver 
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
#include <stdlib.h>

int remote_enable( const ibBoard_t *board, int enable )
{
	int retval;

	if( check_sc( board ) == 0 )
	{
		setIberr( ESAC );
		return -1;
	}

	retval = ioctl( board->fileno, IBSRE, &enable );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	return 0;
}

int _ibsre( ibConf_t *conf, int v )
{
	ibBoard_t *board;
	int retval;

	board = interfaceBoard( conf );

	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		return -1;
	}
	{
	short line_status;
	_iblines( conf, &line_status );
	setIberr( (line_status & BusREN) ? 1:0 );
	}
	retval = remote_enable( board, v );
	if( retval < 0 )
		return retval;

	return 0;
}

int ibsre(int ud, int v)
{
	ibConf_t *conf;
	int retval, result=0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	retval = _ibsre( conf, v );
	if( retval < 0 )
	{
		//fprintf( stderr, "libgpib: ibsre error\n");
		result = func_exit( ud, 1 );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return result;
	}
	result = func_exit( ud, 0 );
//	pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}

int _EnableRemote( ibConf_t *conf, const Addr4882_t addressList[] )
{
	int i;
	ibBoard_t *board;
	uint8_t *cmd;
	int count;
	int retval;

	if( check_addressList( addressList ) == 0 )
		return -1;

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

	retval = remote_enable( board, 1 );
	if( retval < 0 ) return -1;

	if( numofAddresses( addressList ) == 0 )
		return 0;

	cmd = malloc( 16 + 2 * numofAddresses( addressList ) );
	if( cmd == NULL )
	{
		setIberr( EDVR );
		setIbcnt( ENOMEM );
		return -1;
	}

	i = board_send_setup( board, addressList, cmd );

	count = _ibcmd( conf, cmd, i );

	free( cmd );
	cmd = NULL;

	if( count != i )
		return -1;

	return 0;
}


void EnableRemote( int boardID, const Addr4882_t addressList[] )
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
	//pthread_mutex_lock( &semap[boardID]->lock );
	retval = _EnableRemote( conf, addressList );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
		//pthread_mutex_unlock( &semap[boardID]->lock );
		return;
	}
	func_exit( boardID, 0 );
//	pthread_mutex_unlock( &semap[boardID]->lock );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}


