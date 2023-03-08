/***************************************************************************
 ibSic.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibSic.c of the Linux GPIB Package driver 
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

int assert_ifc( ibBoard_t *board, unsigned int usec_duration )
{
	int retval;

	retval = ioctl( board->fileno, IBSIC, &usec_duration );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
	}
	return retval;
}

int _ibsic( ibConf_t *conf )
{
	ibBoard_t *board;

	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		return -1;
	}

	board = interfaceBoard( conf );

	if( check_sc( board ) == 0 )
	{
		setIberr( ESAC );
		return -1;
	}

	return assert_ifc( board, 100 );
}

int ibsic(int ud)
{
	ibConf_t *conf;
	int retval, result=0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	retval = _ibsic( conf );
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

void SendIFC( int boardID )
{
	ibsic( boardID );
}

int request_system_control( ibBoard_t *board, int request_control )
{
	rsc_ioctl_t rsc_cmd;
	int retval;

	rsc_cmd = request_control != 0;
	retval = ioctl( board->fileno, IBRSC, &rsc_cmd );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: IBRSC ioctl failed\n" );
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}
	board->is_system_controller = request_control != 0;
	return 0;
}

int _ibrsc( ibConf_t *conf, int request_control )
{
	int retval;

	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		return -1;
	}
	setIberr( interfaceBoard(conf)->is_system_controller );	
	retval = request_system_control( interfaceBoard( conf ), request_control );
	if( retval < 0 )
		return retval;

	return 0;
}

int ibrsc( int ud, int request_control )
{
	ibConf_t *conf;
	int retval, result=0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	retval = _ibrsc( conf, request_control );
	if( retval < 0 )
	{
		result = func_exit( ud, 1 );
//		pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}

	result = func_exit( ud, 0 );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
//	pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	return result;
}
