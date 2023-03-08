/***************************************************************************
 ibGts.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibGts.c of the Linux GPIB Package driver 
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

int _adsr( ibConf_t *conf, int *status )
{
	ibBoard_t *board;
	int retval;
	wait_ioctl_t cmd;

	board = interfaceBoard( conf );

	if( conf->is_interface == 0 &&
		check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return -1;
	}
	cmd.handle = conf->handle;
	cmd.usec_timeout = conf->settings.usec_timeout;
	cmd.wait_mask = 0;
	cmd.clear_mask = 0;
	cmd.set_mask = 0;
	cmd.set_mask = 0;
	cmd.ibsta = 0;
	if( conf->is_interface == 0 )
	{
		cmd.pad = conf->settings.pad;
		cmd.sad = conf->settings.sad;
	}else
	{
		cmd.pad = NOADDR;
		cmd.sad = NOADDR;
	}

        retval = ioctl( board->fileno, IBSTA, &cmd );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return -1;
	}

	*status = cmd.ibsta;
	return 0;
}

int _ibgts( ibConf_t *conf, int shadow_handshake )
{
	ibBoard_t *board;
	int retval;

	board = interfaceBoard( conf );

	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return -1;
	}

	retval = ioctl( board->fileno, IBGTS, &shadow_handshake );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return -1;
	}

	return 0;
}

int ibgts( int ud, int v )
{
	ibConf_t *conf;
	int retval, result=0;
	char cmdString[100];
	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		return func_exit( ud, 1 );
	}
	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	if((interfaceBoard( conf )->hIntEvent[7]) >= 0x1f) {
	  cmdString[ 0 ] = MTA( conf->settings.pad );
	  _ibcmd( conf, (uint8_t *) cmdString, 1);
	}
	retval = _ibgts( conf, v );
	if( retval < 0 )
	{
		result = func_exit( ud, 1 );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
	        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return result;
		//return func_exit( ud, 1 );
	}
	result = func_exit( ud, 0 );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	return result;
}

