/***************************************************************************
 ibEos.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibEos.c of the Linux GPIB Package driver 
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

int ibeos(int ud, int v)
{
	ibConf_t *conf;
	eos_ioctl_t eos_cmd;
	int retval =0, result=0;

	conf = _func_init( ud, 1, 0 );
	if( conf == NULL )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );

	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	conf->settings.eos = v & 0xff;
	conf->settings.eos_flags = v & 0xff00;

	eos_cmd.eos = conf->settings.eos;
        eos_cmd.eos_flags = conf->settings.eos_flags;

	retval = ioctl( interfaceBoard( conf )->fileno, IBEOS, &eos_cmd );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		//printf("IBEOS ioctl failed\n");
		result = _func_exit( ud, 1, 0, 0, 0, 0, 1 );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}

	result = _func_exit( ud, 0, 0, 0, 0, 0, 1 );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	return result;	
}

int iblcleos( const ibConf_t *conf )
{
	int use_eos, compare8;

	use_eos = conf->settings.eos_flags & REOS;
	compare8 = conf->settings.eos_flags & BIN;

	return config_read_eos( interfaceBoard( conf ), use_eos, conf->settings.eos, compare8 ) ;
}

int config_read_eos( ibBoard_t *board, int use_eos_char, int eos_char,
	int compare_8_bits )
{
	eos_ioctl_t eos_cmd;
	int retval;

	eos_cmd.eos_flags = 0;
	if( use_eos_char )
		eos_cmd.eos_flags |= REOS;
	if( compare_8_bits )
		eos_cmd.eos_flags |= BIN;

	eos_cmd.eos = 0;
	if( use_eos_char )
	{
		eos_cmd.eos = eos_char;
		eos_cmd.eos &= 0xff;
		if( eos_cmd.eos != eos_char )
		{
			setIberr( EARG );
			//printf("invalid eos\n");
			return -1;
		}
	}

	retval = ioctl( board->fileno, IBEOS, &eos_cmd );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		//printf("ibeos ioctl failed\n");
	}
 
	return retval;
}
