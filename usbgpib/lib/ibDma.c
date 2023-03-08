/***************************************************************************
 ibDma.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibDma.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2001,2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
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

int ibdma( int ud, int v )
{
	ibConf_t *conf;
	ibBoard_t *board;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

	board = interfaceBoard( conf );

	return func_exit( ud, 0 );
} /* ibdma */
