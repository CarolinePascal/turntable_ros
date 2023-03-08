/***************************************************************************
 ibgts.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/ibgts.c of the Linux GPIB Package driver 
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
 
#include "u348a.h"

//NATN 
int ibgts( PU348A_DEVEXT board )
{
	int retval;

	retval = adgpib_ibgts( board );

	adgpib_update_status( board, 0 );

	return retval;
}

