/***************************************************************************
 ibPad.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibPad.c of the Linux GPIB Package driver 
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

int _ibpad( ibConf_t *conf, unsigned int address )
{
	ibBoard_t *board;
	int retval;

	board = interfaceBoard( conf );

	if( address > 30 )
	{
		setIberr( EARG );
		fprintf( stderr, "invalid gpib address\n" );
		return -1;
	}

	retval = change_gpib_address( conf, address, conf->settings.sad, 0 );
	if( retval < 0 )
	{
		fprintf( stderr, "failed to change gpib address\n" );
		return -1;
	}

	return 0;
}

int ibpad( int ud, int addr )
{
	ibConf_t *conf;
	int retval, result=0;
	Addr4882_t o_pad = 0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	o_pad = conf->settings.pad;
	retval = _ibpad( conf, addr );
	if( retval < 0 )
	{
		result = func_exit( ud, 1 );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
	        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return result;

	}
	//yuan add 01/17/06
	setIberr( o_pad );
	result = func_exit( ud, 0 );
//        pthread_mutex_unlock( &semap[conf->settings.board]->lock );	  
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}
