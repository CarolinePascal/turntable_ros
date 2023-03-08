/***************************************************************************
 pass_control.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/pass_control.c of the Linux GPIB Package driver 
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

int _pass_control( ibConf_t *conf, unsigned int pad, int sad )
{
	uint8_t cmd;
	int retval;
	int i;

	i = _ReceiveSetup( conf, _mkAddr( pad, sad ) );

	cmd = TCT;
	retval = _ibcmd( conf, &cmd, 1 );
	if( retval < 0 ) return retval;	
	return 0;
}

int ibpct( int ud )
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval, result=0;

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
	retval = _pass_control( conf, conf->settings.pad, conf->settings.sad );
	if( retval < 0 )
	{
		result = func_exit( ud, 1 );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
//		pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return result;
	}
	result = func_exit( ud, 0 );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	return result;
}

void PassControl( int boardID, Addr4882_t address )
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;

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

	//pthread_mutex_lock( &semap[boardID]->lock );
	pthread_mutex_lock( m_lock[boardID] );

	board = interfaceBoard( conf );

	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		func_exit( boardID, 1 );
//		pthread_mutex_unlock( &semap[boardID]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}

	retval = _pass_control( conf, _getPAD( address ), _getSAD( address ) );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
//		pthread_mutex_unlock( &semap[boardID]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	func_exit( boardID, 0 );
//	pthread_mutex_unlock( &semap[boardID]->lock );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}
