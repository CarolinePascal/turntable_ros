/***************************************************************************
 ibsic.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/ibsic.c of the Linux GPIB Package driver 
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
#include "gpib_user.h"
#include "../include/Adllib.h"
#include "u348a.h"
#include "ezusbsys.h"

// assert IFC 100 microseconds.

int ibsic( PU348A_DEVEXT priv, unsigned int usec_duration )
{
	uint8_t retval = 0;

	if( priv->master == 0 )
	{
		printk( "not sytem controller\n" );
		return -1;
	}

	UDServiceFire(priv, VR_GPIB_SIC, 0, 0, 0, NULL, 0);
	//printk("ibsic_ret01\n" );		
	GetGPIBResponse(priv, 1, &retval);
	//printk("ibsic_ret02: %d \n", retval );	
	if(retval != DONE) {
		//printk("ibsic_ret0: %d \n", retval );
		UDServiceFire(priv, VR_GPIB_SIC, 0, 0, 0, NULL, 0);
		GetGPIBResponse(priv, 1, &retval);
		//printk("ibsic_ret1: %d \n", retval );
	}
	return 0;
}

void ibrsc( PU348A_DEVEXT board, int request_control )
{
	U8 retval, rsc = (U8) request_control;
	board->master = request_control != 0;
	
	retval = 0;
	//printk("ibrsc_ret00 \n" );			
	UDServiceFire(board, VR_GPIB_RSC, rsc, 0, 0, NULL, 0);	
	//printk("ibrsc_ret01 \n" );
	GetGPIBResponse(board, 1, &retval);
	//printk("ibrsc_ret02 %d \n", retval );
	if(retval != DONE) {
		UDServiceFire(board, VR_GPIB_RSC, rsc, 0, 0, NULL, 0);
		GetGPIBResponse(board, 1, &retval);
	}
}

