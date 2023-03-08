/***************************************************************************
 autopoll.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/autopoll.c of the Linux GPIB Package driver 
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

#include "../include/Adldev.h"
#include "../include/Adllib.h"
#include "autopoll.h"
#include "gpib_user.h"
#include "gpib_proto.h"

static const unsigned int sp_timeout = 1000000;
 
unsigned int num_status_bytes( const gpib_status_queue_t *dev )
{
	return dev->num_status_bytes;
}


int push_status_byte( gpib_status_queue_t *device, uint8_t poll_byte )
{
	struct list_head *head = &device->status_bytes;
	status_byte_t *status;
	static const unsigned int max_num_status_bytes = 1024;
	int retval;

	if( num_status_bytes( device ) >= max_num_status_bytes )
	{
		uint8_t lost_byte;

		device->dropped_byte = 1;
		retval = pop_status_byte( device, &lost_byte );
		if( retval < 0 ) return retval;
	}

	status = kmalloc( sizeof( status_byte_t ), GFP_KERNEL );
	if( status == NULL ) return -ENOMEM;
    	INIT_LIST_HEAD( &status->list );
	status->poll_byte = poll_byte;

	list_add_tail( &status->list, head );
	
	device->num_status_bytes++;
	return 0;
}

int pop_status_byte( gpib_status_queue_t *device, uint8_t *poll_byte )
{
	struct list_head *head = &device->status_bytes;
	struct list_head *front = head->next;
	status_byte_t *status;

	if( num_status_bytes( device ) == 0 ) return -2;//-EIO;

	if( front == head ) return -1;//-EIO;

	if( device->dropped_byte )
	{
		device->dropped_byte = 0;
		return -7;//-EPIPE;
	}

	status = list_entry( front, status_byte_t, list );	
	*poll_byte = status->poll_byte;

	list_del( front );
	kfree(status);

	device->num_status_bytes--;
	return 0;
}

gpib_status_queue_t * get_spoll_status_queue( PU348A_DEVEXT board, unsigned int pad, int sad )
{
	gpib_status_queue_t *device;
	struct list_head *list_ptr;
	const struct list_head *head = &board->device_list;

	for( list_ptr = head->next; list_ptr != head; list_ptr = list_ptr->next )
	{
		device = list_entry( list_ptr, gpib_status_queue_t, list );
		if( addr_cmp( device->pad, device->sad, pad, sad ) )
			return device;
	}

	return NULL;
}

int get_spoll_status_byte( PU348A_DEVEXT board, unsigned int pad, int sad, unsigned int usec_timeout,
		uint8_t *poll_byte )
{
	gpib_status_queue_t *device;
        int result = 0;
	
        device = get_spoll_status_queue( board, pad, sad );
	if( device == NULL ) {//return -1;//-EINVAL;
	  if( down_interruptible( &board->spoll_mutex ) )
	   {
		return -ERESTARTSYS;
	   }	   
	  result = dvrsp( board, pad, sad, usec_timeout, poll_byte );
	} else {
	 if( down_interruptible( &board->spoll_mutex ) )
	 {
		return -ERESTARTSYS;
	 }
	 if( num_status_bytes( device ) )
	 {
		result = pop_status_byte( device, poll_byte );
	 }else
	 {
		result = dvrsp( board, pad, sad, usec_timeout, poll_byte );
	 }
	}
	up( &board->spoll_mutex );	
	return result;
}

int autopoll_all( PU348A_DEVEXT board )
{
	int retval;

	if( down_interruptible( &board->spoll_mutex ) )
	{
		return -ERESTARTSYS;
	}
	retval = serial_poll_all( board, sp_timeout );
	if( retval < 0 )
	{
		up( &board->spoll_mutex );
		return retval;
	}
	up( &board->spoll_mutex );
	return retval;
}
