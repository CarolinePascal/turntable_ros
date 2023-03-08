/***************************************************************************
 ibrpp.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/ibrpp.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2001, 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 
#include <linux/types.h>
#include "gpib_proto.h"
#include "../include/Adllib.h"
#include "u348a.h"


int ibrpp(PU348A_DEVEXT board, uint8_t *result )
{
	int retval = 0;

	wdt_start( board, board->usec_timeout );
	retval = ibcac( board, 0 );
	if( retval ) return -1;

	if(adgpib_parallel_poll( board, result ) )
	{
		retval = -1;
	}
	wdt_remove(board);
	return retval;
}

int ibppc( PU348A_DEVEXT board, uint8_t configuration )
{

	configuration &= 0x1f;
	write_byte( board, PPR | configuration , AUXMR );
	board->parallel_poll_configuration = configuration;
	
	return 0;
}







