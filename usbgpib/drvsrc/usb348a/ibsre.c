/***************************************************************************
 ibsre.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/ibsre.c of the Linux GPIB Package driver 
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
 
#include <linux/delay.h>
#include "gpib_proto.h"
#include "u348a.h"

int ibsre( PU348A_DEVEXT board, int enable )
{
	if(	board->master == 0 )
	{
		//printk( "not sc\n" );
		return -1;
	}
	
	adgpib_ibsre( board, enable );
	if( !enable )
		udelay(100);
	return 0;
}

