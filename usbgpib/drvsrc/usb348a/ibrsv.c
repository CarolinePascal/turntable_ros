/***************************************************************************
 ibrsv.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from cb7210/cb7210_init.c of the Linux GPIB Package driver 
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

#include "gpib_proto.h"
#include "gpib_user.h"
#include "../include/Adllib.h"
#include "u348a.h"

//slave requests service
int ibrsv( PU348A_DEVEXT board, uint8_t poll_status )
{
	int status = ibstatus( board );

	if( ( status & CIC ) )
	{
		printk("ibsrv only for Non-CIC\n");
		return -1;//-EINVAL;
	}
	write_byte(board, poll_status, SPMR);//set device status 

	return 0;
}
