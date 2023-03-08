/***************************************************************************
 ibRsv.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibRsv.c of the Linux GPIB Package driver 
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

int _ibrsv( ibConf_t *conf, int v )
{
	ibBoard_t *board;
	unsigned short status_byte = v;
	int retval;
	
	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		return -1;
	}

	board = interfaceBoard( conf );
	setIberr( ask_board_rsv( board ) );
	retval = ioctl( board->fileno, IBRSV, &status_byte );
	if( retval < 0 )
	{
		return retval;
	}

	return 0;
}

int ibrsv( int ud, int v )
{
	ibConf_t *conf;
	int retval, result=0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	pthread_mutex_lock( m_lock[conf->settings.board] );
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	retval = _ibrsv( conf, v );
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

