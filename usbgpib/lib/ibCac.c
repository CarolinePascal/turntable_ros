/***************************************************************************
 ibCac.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibCac.c of the Linux GPIB Package driver 
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

int _ibcac( ibConf_t *conf, int synchronous )
{
	ibBoard_t *board;
	int retval;
    
	board = interfaceBoard( conf );

	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return -1;
	}
        retval = ioctl( board->fileno, IBCAC, &synchronous );
	if(retval < 0)
	{
		switch( errno )
		{
			default:
				setIberr( EDVR );
				break;
		}
		 setIberr( EDVR );
		 setIbcnt( errno );
		 return -1;
	}
	return 0;
}

int ibcac( int ud, int synchronous )
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval, result =0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		return func_exit( ud, 1 );
	}

	board = interfaceBoard( conf );

	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return func_exit( ud, 1 );
	}
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[conf->settings.board] );
	retval = ioctl( board->fileno, IBCAC, &synchronous );
	if( retval < 0 && synchronous  )
	{
		synchronous = 0;
		retval = ioctl( board->fileno, IBCAC, &synchronous );
	}
	if(retval < 0)
	{
		switch( errno )
		{
			default:
				setIberr( EDVR );
				break;
		}
		result = func_exit( ud, 1 );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
	        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		return result;
	}

	result = func_exit( ud, 0 );
        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}
