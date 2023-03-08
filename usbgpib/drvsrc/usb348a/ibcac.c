/***************************************************************************
 ibcac.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/ibcac.c of the Linux GPIB Package driver 
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
 
#include "../include/Adllib.h"
#include "gpib_user.h"
#include "../include/gpib_ioctl.h"
#include "gpib_proto.h"
#include "u348a.h"


int ibcac( PU348A_DEVEXT board, int sync )
{
	int retval;

	retval = adgpib_ibcac( board, sync );

	return retval;
}




