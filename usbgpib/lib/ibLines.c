/***************************************************************************
 ibLines.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibLines.c of the Linux GPIB Package driver 
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

int _iblines( ibConf_t *conf, short *line_status )
{
	int retval;
	ibBoard_t *board;

	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		return -1;
	}

	board = interfaceBoard( conf );

	retval = ioctl( board->fileno, IBLINES, line_status );
	if( retval < 0 )
	{
		switch( errno )
		{
			default:
				setIbcnt( errno );
				setIberr( EDVR );
				break;
		}
		return -1;
	}
	return 0;
}

int iblines( int ud, short *line_status )
{
	ibConf_t *conf;
	int retval;

	conf = _func_init( ud, 1, 1 );
	if( conf == NULL )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );
        *line_status = 0;
	retval = _iblines( conf, line_status );
	if( retval < 0 )
	{
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );
	}

	return _func_exit( ud, 0, 0, 0, 0, 0, 1 );
}

void TestSRQ( int boardID, short *result )
{
	short line_status;

	ibConf_t *conf;
	int retval;

	conf = _func_init( boardID, 1, 0 );
	if( conf == NULL )
	{
		_func_exit( boardID, 1, 0, 0, 0, 0, 1 );
		return;
	}

	retval = _iblines( conf, &line_status );
	if( retval < 0 )
	{
		_func_exit( boardID, 1, 0, 0, 0, 0, 1 );
		return;
	}

	if( ( line_status & ValidSRQ ) == 0 )
	{
		setIberr( ECAP );
		_func_exit( boardID, 1, 0, 0, 0, 0, 1 );
		return;
	}

	if( line_status & BusSRQ )
	{
		*result = 1;
	}else
		*result = 0;

	_func_exit( boardID, 0, 0, 0, 0, 0, 1 );
}
