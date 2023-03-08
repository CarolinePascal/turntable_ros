/***************************************************************************
 autopoll.h
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/autopoll.h of the Linux GPIB Package driver
 							by Frank Mori Hess      
 Copyright (C) 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 
#ifndef GPIB_AUTOPOLL_H
#define GPIB_AUTOPOLL_H

#include "../include/Adldev.h"

gpib_status_queue_t * get_spoll_status_queue( PU348A_DEVEXT board, unsigned int pad, int sad );
unsigned int num_status_bytes( const gpib_status_queue_t *dev );
int push_status_byte( gpib_status_queue_t *device, uint8_t poll_byte );
int pop_status_byte( gpib_status_queue_t *device, uint8_t *poll_byte );
gpib_status_queue_t * get_gpib_status_queue( PU348A_DEVEXT board, unsigned int pad, int sad );
int get_spoll_status_byte( PU348A_DEVEXT board, unsigned int pad, int sad, unsigned int usec_timeout, uint8_t *poll_byte );
int autopoll_all( PU348A_DEVEXT board );

#endif // GPIB_AUTOPOLL_H
