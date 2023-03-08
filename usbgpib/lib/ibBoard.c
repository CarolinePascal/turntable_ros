/***************************************************************************
 ibBoard.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibBoard.c of the Linux GPIB Package driver 
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
 
#include <stdio.h>
#include <stdlib.h>

#include "ib_internal.h"

#include <fcntl.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <string.h>
#include <sys/types.h>
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>

ibBoard_t ibBoard[ GPIB_MAX_NUM_BOARDS ];
//extern pthread_mutex_t* m_lock[GPIB_MAX_NUM_BOARDS];

void init_ibboard(ibBoard_t *board)
{
	int vi=0;
	strcpy(board->board_type, "");
	for(vi=0; vi<6;vi++)
		board->base[vi] = 0;
	board->irq = 0;
	board->dma = 0;
	board->pci_bus = -1;
	board->pci_slot = -1;
	board->fileno = -1;
	strcpy(board->device, "");
	board->open_count = 0;
	board->is_system_controller = 0;
	board->use_event_queue = 0;
	board->autospoll = 0;
	for(vi=0; vi<8;vi++)
		board->hIntEvent[vi] = 0;
	board->hIntEvent[7] = 0x1f;
}

int configure_autospoll(ibConf_t *conf, int enable)
{
	autospoll_ioctl_t spoll_enable = enable != 0;
	int retval = 0;
	ibBoard_t *board = interfaceBoard(conf);

	setIberr( board->autospoll? 1:0 );
	if((spoll_enable && board->autospoll == 0) ||
		(spoll_enable == 0 && board->autospoll))
	{
		retval = ioctl(interfaceBoard(conf)->fileno, IBAUTOSPOLL, &spoll_enable);
		if(retval)
		{
			fprintf(stderr, "autospoll ioctl error %i\n", retval);
		}else
		{
			board->autospoll = enable != 0;
		}
	}
	board->autospoll = ask_autopoll( board );
	return retval;
}

int ibBoardOpen( ibBoard_t *board )
{
	int fd, sc=0;
	int flags = 0, err;

	if( board->fileno >= 0 ) return 0;
	if( ( fd = open( board->device, O_RDWR | flags ) ) < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		//fprintf( stderr, "libgpib: ibBoardOpen failed to open device file %s\n", board->device);
		////perror( "libgpib" );
		return -1;
	}
	board->fileno = fd;
//get resource???
	board->open_count++;
	err = pthread_mutex_lock( m_lock[board->ud]);
/*{
      semBuf.sem_num = 0;
       semBuf.sem_op = -1;
       semBuf.sem_flg = SEM_UNDO;
       if (semop(shr_sem[0], &semBuf, 1) != 0)
                printf(" RC_LOCK_ERROR %d\n", errno);
}*/
        sc = check_sc( board );
	if(board->is_system_controller) 
	{
  	    if(!sc)
	    {
		_ibrsc( ibConfigs[ board->ud ], 1 );
		sc = 1;
	    }	
	}
	if( ibConfigs[board->ud]->settings.bus_timming != ask_board_t1_delay( board ))
	set_t1_delay( board,  ibConfigs[board->ud]->settings.bus_timming);
	change_gpib_address( ibConfigs[ board->ud ], ibConfigs[board->ud]->settings.pad, ibConfigs[board->ud]->settings.sad, 1 );
	if(board->is_system_controller) {
	 board->autospoll = ask_autopoll( board );
	 if(board->defautospoll)
		configure_autospoll( ibConfigs[ board->ud ], 1 );
	 else
		configure_autospoll( ibConfigs[ board->ud ], 0 );
	} else {
		//configure_autospoll( ibConfigs[ board->ud ], 0 );
		if(sc)
  		 _ibrsc( ibConfigs[ board->ud ], 0 );
	}
	iblcleos(ibConfigs[board->ud]);
	err = pthread_mutex_unlock( m_lock[board->ud] );
/*semBuf.sem_num = 0;
       semBuf.sem_op  = 1;
       semBuf.sem_flg = SEM_UNDO;

       if (semop(shr_sem[0], &semBuf, 1) != 0)
           printf(" RC_UNLOCK_ERROR\n");*/
	setIberr( 0 );
	return 0;
}

int ibBoardClose( ibBoard_t *board )
{

	if( board->open_count == 0 )
	{
		fprintf( stderr, "board has not been opened yet\n");
		return -1;
	}

	board->open_count--;
	if( board->open_count > 0 )
		return 0;

	if( board->fileno >= 0 )
	{
		close( board->fileno );
		board->fileno = -1;
	}

	return 0;
}







