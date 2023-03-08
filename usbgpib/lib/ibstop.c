/***************************************************************************
 ibstop.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibstop.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2003 by Frank Mori Hess <fmhess@users.sourceforge.net>
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
#include <pthread.h>

int _ibstop( ibConf_t *conf )
{
	int retval;

	pthread_mutex_lock( &conf->async.lock );

	if( (conf->async.in_progress == 0) || (conf->async.ibsta & CMPL))
	{
		pthread_mutex_unlock( &conf->async.lock );
		return 0;
	}
	conf->async.abort = 1;

	abort_aio( conf );

	pthread_mutex_unlock( &conf->async.lock );

	retval = aio_thread_join( &conf->async );
	if( retval )
	{
		return -1;
	}

	pthread_mutex_lock( &conf->async.lock );

	conf->async.abort = 0;
	conf->async.in_progress = 0;
	pthread_mutex_unlock( &conf->async.lock );
	setIbsta( conf->async.ibsta );
	setIbcnt(conf->async.ibcntl );
	setIberr( EABO );
	return 1;
}

int ibstop( int ud )
{
	ibConf_t *conf;
	int retval;

	conf = _func_init( ud, 1, 1 );
	if( conf == NULL )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );

	retval = _ibstop( conf );
	if( retval < 0 )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );

	//yuan modify 01/13/12
	return _func_exit( ud, 0, 0, 1, 0, CMPL, 1 );
	//return _func_exit( ud, 0, 0, 0, 0, CMPL, 1 );
}
