/***************************************************************************
 ibWait.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibWait.c of the Linux GPIB Package driver 
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
 
#include "ib_internal.h"
#include <pthread.h>
#include <sys/time.h>
#include <sched.h>

void fixup_status_bits( const ibConf_t *conf, int *status )
{
	const int board_wait_mask = board_status_mask ;//& ~ERR;
	const int device_wait_mask = device_status_mask;// & ~ERR;
	ibBoard_t *board;

	if( conf->is_interface == 0 )
	{
		*status &= device_wait_mask;
	}else
	{
		*status &= board_wait_mask;
		board = interfaceBoard(conf);
		//yuan add 02/26/08
		if((*status & REM) && (*status & LACS)) 
			board->rem_sta = 1;
		if(!(board->rem_sta) && (*status & REM) && !(*status & LACS)) 
			*status &= ~REM;		
	}
}

int _ibwait( ibConf_t *conf, int wait_mask, int clear_mask, int set_mask, int *status )
{
	ibBoard_t *board;
	int retval;
	wait_ioctl_t cmd;

	board = interfaceBoard( conf );

/*	if( conf->is_interface == 0 &&
		check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return -1;
	}
*/
	cmd.handle = conf->handle;
	cmd.usec_timeout = conf->settings.usec_timeout;
	cmd.wait_mask = wait_mask;
	//yuan add 02/27/08
	if(conf->async.in_progress)
			clear_mask |= RQS;
	cmd.clear_mask = clear_mask;
	cmd.set_mask = set_mask;
	cmd.set_mask = 0;
	cmd.ibsta = 0;
	fixup_status_bits( conf, &cmd.wait_mask );
	if( conf->is_interface == 0 )
	{
		cmd.pad = conf->settings.pad;
		cmd.sad = conf->settings.sad;
	}else
	{
		cmd.pad = NOADDR;
		cmd.sad = NOADDR;
	}

	if( wait_mask != cmd.wait_mask )
	{
		setIberr( EARG );
		return -1;
	}

	retval = ioctl(board->fileno, IBWAIT, &cmd);
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return -1;
	}
	if( conf->end ) {
		//*status |= END;
		//printf("read wait end\n");
		cmd.ibsta |= END;
	} else {
		cmd.ibsta &= (~END);
	}
	pthread_mutex_lock( &conf->async.lock );	
////	WaitForSingleObject(conf->async.lock, INFINITE);
	if( conf->async.in_progress )
	{
	
            if( conf->async.ibsta & CMPL ) {
		conf->async.in_progress = 0;
		//yuan add 09/25/06
		conf->async.opstart = 0;
		//ResetEvent (conf->async.startE);
		cmd.ibsta |= conf->async.ibsta;
//printf("wait complete %x %x\n", cmd.ibsta, conf->async.iberr);
		if( !(conf->async.ibsta & END) ) {
			cmd.ibsta &= (~END);
		} 
		//yuan add 07/08/09
		pthread_mutex_unlock( &conf->async.lock );
		aio_thread_join( &conf->async );
		pthread_mutex_lock( &conf->async.lock );	
		setIbcnt( conf->async.ibcntl );
		setIberr( conf->async.iberr );
	    } else {
		cmd.ibsta &= (~CMPL); 
		cmd.ibsta &= (~END); 
	    }
            if( conf->async.ibsta & ERR )
	    {
		setIbsta( conf->async.ibsta );
		pthread_mutex_unlock( &conf->async.lock );
		//ReleaseMutex(conf->async.lock);
		return -1;
	     }				
	}
	////ReleaseMutex(conf->async.lock);
	fixup_status_bits( conf, &cmd.ibsta );
	setIbsta( cmd.ibsta );
	pthread_mutex_unlock( &conf->async.lock );
	*status = cmd.ibsta;
	//if (!board->is_system_controller & (cmd.ibsta & CIC))
	//	board->is_system_controller = 1;
	return 0;
}

int _ibwait_a( ibConf_t *conf, int wait_mask, int clear_mask, int set_mask, int *status )
{
	ibBoard_t *board;
	wait_ioctl_t cmd;
	int retval=0;
	
	board = interfaceBoard( conf );
	cmd.handle = conf->handle;
	cmd.usec_timeout = conf->settings.usec_timeout;
	cmd.wait_mask = wait_mask;
	//yuan add 02/27/08
	if(conf->async.in_progress)
		clear_mask |= RQS;
	cmd.clear_mask = clear_mask;
	cmd.set_mask = set_mask;
	cmd.set_mask = 0;
	cmd.ibsta = 0;
	fixup_status_bits( conf, &cmd.wait_mask );
	if( conf->is_interface == 0 )
	{
		cmd.pad = conf->settings.pad;
		cmd.sad = conf->settings.sad;
	}else
	{
		cmd.pad = NOADDR;
		cmd.sad = NOADDR;
	}
	if( wait_mask != cmd.wait_mask )
	{
		setIberr( EARG );
		return -1;
	}
	retval = ioctl(board->fileno, IBWAIT, &cmd);
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return -1;
	}
	if( conf->end ) {
		//*status |= END;
		cmd.ibsta |= END;
	} else {
		cmd.ibsta &= (~END);
	}
	pthread_mutex_lock( &conf->async.lock );
	////WaitForSingleObject(conf->async.lock, INFINITE);
	if( conf->async.in_progress )
	{
		
		if( conf->async.ibsta & CMPL ) {
			cmd.ibsta |= conf->async.ibsta;
			if( !(conf->async.ibsta & END) ) {
				cmd.ibsta &= (~END);
			} //else
			//yuan add 07/08/09
			pthread_mutex_unlock( &conf->async.lock );
			aio_thread_join( &conf->async );
			pthread_mutex_lock( &conf->async.lock );			
			setIbcnt( conf->async.ibcntl );
			setIberr( conf->async.iberr );
		} else {
			cmd.ibsta &= (~CMPL); 
			cmd.ibsta &= (~END); 
		}
		if( conf->async.ibsta & ERR )
		{
			setIbsta( conf->async.ibsta );
			////ReleaseMutex(conf->async.lock);
			pthread_mutex_unlock( &conf->async.lock );
			return -1;
		}				
	}
	///ReleaseMutex(conf->async.lock);
	fixup_status_bits( conf, &cmd.ibsta );
	setIbsta( cmd.ibsta );
	pthread_mutex_unlock( &conf->async.lock );
	*status = cmd.ibsta;
	//if (!board->is_system_controller & (cmd.ibsta & CIC))
	//	board->is_system_controller = 1;	
	return 0;
}

int ibwait( int ud, int mask )
{
	ibConf_t *conf;
	int retval;
	int status;
	int clear_mask;
	int error = 0;
	double take_msec, timeout_u=0.0;
	struct timeval tv_start, tv_end;
  	struct timezone tz;

	tz.tz_minuteswest = 0;
	tz.tz_dsttime = 0;

	conf = _func_init( ud, 1, 0 );
	if( conf == NULL )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );

	clear_mask = mask & ( DTAS | DCAS );
	//yuan add 10/20/06
	if(!conf->settings.usec_timeout) {
		mask &= ~TIMO;
		//timeout_u = 0xffffffff;
	} else
		timeout_u = (double) ((conf->settings.usec_timeout>=LINETIMEOUT)? (conf->settings.usec_timeout+ADDTIMEOUT):conf->settings.usec_timeout);
timeout_u /= 1000;
      gettimeofday( &tv_start, &tz );
do {
	 retval = _ibwait( conf, mask, clear_mask, 0, &status );
	 if( retval < 0 )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );
	 if(mask & TIMO) {
	   gettimeofday( &tv_end, &tz );
	   take_msec = (tv_end.tv_sec*1000+((double)(tv_end.tv_usec)/1000.0))\
			-(tv_start.tv_sec*1000+((double)(tv_start.tv_usec)/1000.0));
	   if(take_msec > timeout_u)
           {//conf->settings.usec_timeout)
		  status |= TIMO;
		  //yuan add 03/30/07
		  setIbsta( status );
	   }
	   //if(!(status & mask))
	   usleep(1000);
	       //sched_yield();
	 }
else
usleep(100);
//sched_yield();
} while (mask && (!(status & mask)));

/****	if(conf->async.in_progress)
	{
		if( gpib_aio_join( &conf->async ) )
		 	error++;
		pthread_mutex_lock( &conf->async.lock );
		if( conf->async.ibsta & CMPL )
			conf->async.in_progress = 0;
		setIbcnt( conf->async.ibcntl );
		setIberr( conf->async.iberr );
		if( conf->async.ibsta & ERR )
		{
			error++;
		}
		pthread_mutex_unlock( &conf->async.lock );
		if(error && (ThreadIbsta() & ERR) == 0)
		{
			status |= ERR;
			setIbsta(status);
		}
	}
***/
	_func_exit( ud, error, 0, 1, 0, 0, 1 );

	return status;
}

void WaitSRQ( int boardID, short *result )
{
	ibConf_t *conf;
	int retval;
	int wait_mask;
	int status;
double timeout_u;
	conf = _func_init( boardID, 1, 0 );
	if( conf == NULL )
	{
		_func_exit( boardID, 1, 0, 0, 0, 0, 1 );
		return;
	}

	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		_func_exit( boardID, 1, 0, 0, 0, 0, 1 );
		return;
	}
    	if(conf->settings.usec_timeout) {
		timeout_u = (double) ((conf->settings.usec_timeout>=LINETIMEOUT)? (conf->settings.usec_timeout+ADDTIMEOUT):conf->settings.usec_timeout);
	///	QueryPerformanceFrequency(&freq);
	////	QueryPerformanceCounter(&start_count);	//gettimeofday
		wait_mask = SRQI | TIMO;
	} else //yuan add 10/24/06
		wait_mask = SRQI;
////	wait_mask = SRQI | TIMO;
	do {
	retval = _ibwait( conf, wait_mask, 0, 0, &status );
	if( retval < 0 )
	{
		_func_exit( boardID, 1, 0, 0, 0, 0, 1 );
		return;
	}
	 if(conf->settings.usec_timeout) {
	    /****QueryPerformanceCounter(&current_count);
	    take_msec = ((double)(current_count.QuadPart-start_count.QuadPart) / ((double) freq.QuadPart)) * 1000000;
		if(take_msec > timeout_u) {//conf->settings.usec_timeout)
		  status |= TIMO;
  		  //yuan add 03/30/07
		  setIbsta( status );
		}****/
	 }
	    //yuan add 09/25/06
	if(status & wait_mask)
	 break;
	else 
	  sleep(0);//Sleep(0);
	} while (1);//(!(status & wait_mask));

	if( ThreadIbsta() & SRQI ) *result = 1;
	else *result = 0;

	_func_exit( boardID, 0, 0, 0, 0, 0, 1 );
}
