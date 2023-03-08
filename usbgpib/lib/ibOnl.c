/***************************************************************************
 ibOnl.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibOnl.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2001,2002,2003 by Frank Mori Hess <fmhess@users.sourceforge.net>
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
#include <stdlib.h>

int board_online( ibBoard_t *board, int online )
{
	if( online )
	{
		if( ibBoardOpen( board ) < 0 )
			return -1;
	}else
	{
		ibBoardClose( board );
	}

	return 0;
}

//yuan add 08/30/05
int Board_Reset(ibBoard_t *board)
{
    online_ioctl_t online_cmd;
    int retval = 0;

    online_cmd.online = 2;
    retval = ioctl( board->fileno, IBONL, &online_cmd );
    if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		printf( "board reset error!\n");
		return -1;
	}
	return retval;
}

int conf_online( ibConf_t *conf, int online )
{
	ibBoard_t *board;
	int retval;

	if( ( online && conf->board_is_open ) ||
		( online == 0 && conf->board_is_open == 0 ) )
		return 0;
	board = interfaceBoard( conf );
	retval = board_online( board, online );
	if( retval < 0 ) return retval;
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[conf->settings.board] );
	if( online )
	{
		retval = open_device_handle( conf );
	}else
	{
		retval = close_device_handle( conf );
	}
	if( retval < 0 ) 
	{
//		pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return retval;
	}

	conf->board_is_open = online != 0;
	//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return 0;
}

int set_config_default( ibConf_t *conf )
{
	int retval;

	retval = _ibpad( conf, conf->defaults.pad );
	if( retval < 0 ) return retval;
	retval = _ibsad( conf, conf->defaults.sad );
	if( retval < 0 ) return retval;
	//retval = my_ibbna( conf, conf->defaults.board );
	//if( retval < 0 ) return retval;
	conf->settings.usec_timeout = conf->defaults.usec_timeout;
	//yuan modify 10/20/06
	conf->settings.spoll_usec_timeout = conf->defaults.spoll_usec_timeout;
	//conf->settings.spoll_usec_timeout = conf->defaults.usec_timeout;
	conf->settings.ppoll_usec_timeout = conf->defaults.ppoll_usec_timeout;
	//conf->settings.ppoll_usec_timeout = conf->defaults.usec_timeout;
	conf->settings.eos = conf->defaults.eos;
	conf->settings.eos_flags = conf->defaults.eos_flags;
	conf->settings.ppoll_config = conf->defaults.ppoll_config;
	_ibeot( conf, conf->defaults.send_eoi );
	conf->settings.local_lockout = conf->defaults.local_lockout;
	conf->settings.local_ppc = conf->defaults.local_ppc;
	conf->settings.readdr = conf->defaults.readdr;
	conf->settings.bus_timming = conf->defaults.bus_timming;
	return 0;
}

int ibonl( int ud, int onl )
{
	ibConf_t *conf;
	int retval, result=0;
	ibBoard_t *board;
	int status=0;

	setIbcnt( 0 );

	conf = _func_init( ud, 1, 1 );

	if( conf == NULL ) 
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );
	conf->timed_out = 0;
	conf->end = 0;
	retval = _ibstop( conf );
	if( retval < 0 )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );

	pthread_mutex_lock( m_lock[conf->settings.board] );

	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
        board = interfaceBoard( conf );
	if( onl )
	{
		if( conf->is_interface )
			//yuan add 08/30/05
			 Board_Reset(board);   
		retval = set_config_default( conf );
		if( retval < 0 ) 
		{
			result = func_exit( ud, 1 );
			pthread_mutex_unlock( m_lock[conf->settings.board] );
			//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
			return result;
		}
		//else return func_exit( ud, 0 );
		else {
		   //////yuan add 02/19/08
		    if(conf->is_interface) {
			 if( conf->settings.bus_timming != ask_board_t1_delay( board ))
			   set_t1_delay( board,  conf->settings.bus_timming);
			 //yuan add 05/17/06
			 if(board->is_system_controller) 
				_ibrsc( conf, 1 );
			 if(board->is_system_controller) {
				board->autospoll = ask_autopoll( board );
			    if(board->defautospoll)
				  configure_autospoll( conf, 1 );
			    else
				configure_autospoll( conf, 0 );
			 } else {
				//configure_autospoll( ibConfigs[ board->ud ], 0 );
				_ibrsc( conf, 0 );
			 }
			 //yuan add 03/24/06
			 iblcleos(conf);
		    } //interface
		    set_ibLon( interfaceBoard( conf ), 1);
		    result = func_exit( ud, 0 );
		    //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		    pthread_mutex_unlock( m_lock[conf->settings.board] );
		    return result;
		}
	}
        if( !conf->is_interface ) {
  		status = _func_exit( ud, 0, 0, 0, 0, CMPL, 1 );
    	}
	//board = interfaceBoard( conf );

	//status = _func_exit( ud, 0, 0, 0, 0, CMPL, 1 );

	if( onl == 0 )
	{
		retval = close_device_handle( conf );
		if( conf->is_interface )
		 {
			 //yuan add 08/30/05
			 Board_Reset(board);     
			 board->assert_ifc = 0;
			 //yuan add 05/27/08
			 board->rem_sta = 0;
			 set_config_default( conf );
		         if( conf->settings.bus_timming != ask_board_t1_delay( board ))
			   set_t1_delay( board,  conf->settings.bus_timming);
			 //yuan add 05/17/06
			 if(board->is_system_controller) 
				_ibrsc( conf, 1 );
			 if(board->is_system_controller) {
				board->autospoll = ask_autopoll( board );
				if(board->defautospoll)
					configure_autospoll( conf, 1 );
				else
					configure_autospoll( conf, 0 );

			 } else {
				//configure_autospoll( ibConfigs[ board->ud ], 0 );
				_ibrsc( conf, 0 );
			 }
			 //yuan add 03/24/06
			 iblcleos(conf);
			 //configure_autospoll( conf, 0 );
		} //if it is a interface
	} else {
			retval = 0;
			//..ibBoardClose( board );
	}
	//}

	pthread_mutex_unlock( m_lock[conf->settings.board] );

//	pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	if( conf->is_interface ) {
		status = _func_exit( ud, 0, 0, 0, 0, CMPL, 1 );
	}
	if( retval < 0 )
	{
		printf( "failed to close device handle!\n" );
		setIberr( EDVR );
		setIbcnt( errno );
		status |= ERR;
		setIbsta( status );
		sync_globals();
		return status;
	}

	if( ud >= GPIB_MAX_NUM_BOARDS )
	{
		if(ibConfigs[ ud ]) {
			free( ibConfigs[ ud ] );
			ibConfigs[ ud ] = NULL;
		}
	}
	return status;
}


