/***************************************************************************
 ibLoc.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibLoc.c of the Linux GPIB Package driver 
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
#include "ibP.h"
#include <stdlib.h>

int ibloc(int ud)
{
	ibConf_t *conf;
	ibBoard_t *board;
	uint8_t cmd[32];
	unsigned int i;
	ssize_t count;
	int retval, result=0;

	conf = _func_init(ud, 1, 0);
	if( conf == NULL )
		return func_exit( ud, 1 );

	pthread_mutex_lock( m_lock[conf->settings.board] );
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	board = interfaceBoard( conf );

	if( conf->is_interface )
	{
		unsigned short loc=0;
		retval = ioctl( board->fileno, IBLOC, &loc );
		if( retval < 0 )
		{
			//printf( "IBLOC ioctl failed\n" );
			setIberr( EDVR );
			setIbcnt( errno );
			result = func_exit( ud, 1 );
			pthread_mutex_unlock( m_lock[conf->settings.board] );
		        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
			return result;
		}
	}else
	{
		i = create_sent_cmd_string( conf, cmd );
		if( i < 0 )
		{
			setIberr( EDVR );
			result = func_exit( ud, 1 );
			pthread_mutex_unlock( m_lock[conf->settings.board] );
		        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
			return result;
		}
		cmd[ i++ ] = GTL;
		count = _ibcmd( conf, cmd, i);
		if(count != i)
		{
			result = func_exit( ud, 1 );
			pthread_mutex_unlock( m_lock[conf->settings.board] );
		        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
			return result;
		}
	}

	result = func_exit( ud, 0 );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
//        pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	return result;
}

void EnableLocal( int boardID, const Addr4882_t addressList[] )
{
	int i;
	ibConf_t *conf;
	ibBoard_t *board;
	uint8_t *cmd;
	int count;
	int retval;
	
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
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	board = interfaceBoard( conf );

	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
//		pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return;
	}

	if( numofAddresses( addressList ) == 0 )
	{
		retval = remote_enable( board, 0 );
		if( retval < 0 ) func_exit( boardID, 1 );		
		else func_exit( boardID, 0 );
		pthread_mutex_unlock( m_lock[boardID] );
		//pthread_mutex_lock( &semap[conf->settings.board]->lock );
		return;
	}

	cmd = malloc( 16 + 2 * numofAddresses( addressList ) );
	if( cmd == NULL )
	{
		setIberr( EDVR );
		setIbcnt( ENOMEM );
		func_exit( boardID, 1 );
//		pthread_mutex_lock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}

	i = board_send_setup( board, addressList, cmd );
	cmd[ i++ ] = GTL;

	count = _ibcmd( conf, cmd, i );

	free( cmd );
	cmd = NULL;

	if(count != i)
	{
		func_exit( boardID, 1 );
		//pthread_mutex_lock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	return;
}

