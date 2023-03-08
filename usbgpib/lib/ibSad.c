/***************************************************************************
 ibSad.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibSad.c of the Linux GPIB Package driver 
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

int _ibsad( ibConf_t *conf, int address )
{
	ibBoard_t *board;
	int sad = address - sad_offset;
	int retval;

        if((sad<0) && (conf->settings.sad<0))
		return 0;     
	board = interfaceBoard( conf );
	if( sad > 30 )
	{
		setIberr( EARG );
		return -1;
	}

	retval = change_gpib_address( conf, conf->settings.pad, sad, 0 );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: failed to change gpib address\n" );
		return -1;
	}
	return 0;
}

int ibsad( int ud, int v )
{
	ibConf_t *conf;
	int retval, result=0;
	Addr4882_t o_sad = 0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	//yuan add 01/17/06
	pthread_mutex_lock( m_lock[conf->settings.board] );
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	o_sad = conf->settings.sad;
	retval = _ibsad( conf, v );
	if( retval < 0 )
	{
		result = func_exit( ud, 1 );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
	        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return result;		
	}
	//yuan add 01/17/06
	setIberr( o_sad );
	result = func_exit( ud, 0 );
        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;		
}
