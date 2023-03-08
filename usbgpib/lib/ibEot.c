/***************************************************************************
 ibEot.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibEot.c of the Linux GPIB Package driver 
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
#include <sys/ioctl.h>

void _ibeot( ibConf_t *conf, int send_eoi )
{
	setIberr( conf->settings.send_eoi );
	if(send_eoi)
		conf->settings.send_eoi = 1;
	else
		conf->settings.send_eoi = 0;
}

int ibeot( int ud, int send_eoi )
{
	ibConf_t *conf;
	int result=0;

	conf = _func_init( ud, 1, 0 );
	if( conf == NULL )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );

	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );

	_ibeot( conf, send_eoi );

	result = _func_exit( ud, 0, 0, 0, 0, 0, 1 );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
//        pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	return result;
}
