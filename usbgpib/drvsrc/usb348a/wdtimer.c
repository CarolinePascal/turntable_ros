/***************************************************************************
 wdtimer.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/ostimer.c of the Linux GPIB Package driver 
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

#include "../include/Adldev.h"
#include "../include/Adllib.h"
#include "gpib_user.h"
/*
 * Timer functions
 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
void timeout_routine( unsigned long arg )
{
	PU348A_DEVEXT board = (PU348A_DEVEXT) arg;

	set_bit( TIMO_NUM, &board->status );
	//printk("timeout\n");
	///wake_up_interruptible( &board->wait );
}
#else
void timeout_routine( struct timer_list *t )
{
	PU348A_DEVEXT board = (PU348A_DEVEXT) from_timer(board, t, adtimer);

	set_bit( TIMO_NUM, &board->status );
	//printk("timeout\n");
	///wake_up_interruptible( &board->wait );
}
#endif

void wdt_start( PU348A_DEVEXT board, unsigned int usec_timeout )
{
	if( timer_pending( &board->adtimer ) )
	{
		//printk("timer is runnning\n");
		return;
	}
	clear_bit( TIMO_NUM, &board->status );

	if( usec_timeout > 0 )
	{
		board->adtimer.expires = 1+jiffies + usecs_to_jiffies( usec_timeout );//expired time ticks
		board->adtimer.function = timeout_routine;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)		
		board->adtimer.data = (unsigned long) board;
#endif		
		add_timer( &board->adtimer ); /* add timer */
	}
}

void wdt_remove( PU348A_DEVEXT board )
{
	if( timer_pending( &board->adtimer ) )
		del_timer_sync( &board->adtimer );
}

int io_timed_out( PU348A_DEVEXT board )
{
	if( test_bit( TIMO_NUM, &board->status ) ) return 1;
	return 0;
}

