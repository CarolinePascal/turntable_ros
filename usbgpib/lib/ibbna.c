/***************************************************************************
 ibbna.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibbna.c of the Linux GPIB Package driver 
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
 
#include "ib_internal.h"

int _ibbna( ibConf_t *conf, unsigned int new_board_index )
{
	ibBoard_t *board, *new_board;
	ibConf_t *board_conf;
	int retval;
	int old_board_index;

	board = interfaceBoard( conf );

	if( conf->is_interface )
	{
		setIberr( EARG );
		return -1;
	}
	retval = close_device_handle( conf );
	if( retval < 0 )
	{
		setIberr( EDVR );
		return -1;
	}

	board_conf = &ibFindConfigs[ new_board_index ];
	if( board_conf->is_interface == 0 )
	{
		setIberr( EARG );
		return -1;
	}
	//yuan modify 09/20/06
	/***new_board = interfaceBoard( board_conf );
	if( check_cic( new_board ) == 0 )
	{
		assert_ifc( new_board, 100 );
		new_board->assert_ifc = 1;
//		setIberr( ECIC );
//		return -1;
	}***/

	old_board_index = conf->settings.board;
	conf->settings.board = board_conf->settings.board;

	if( ibBoardOpen( interfaceBoard( conf ) ) < 0 )
	{
		setIberr( ENEB );
		return -1;
	}

	//yuan modify 09/20/06
	new_board = interfaceBoard( board_conf );
	if( check_cic( new_board ) == 0 )
	{
		assert_ifc( new_board, 100 );
		new_board->assert_ifc = 1;
//		setIberr( ECIC );
//		return -1;
	}
	retval = open_device_handle( conf );
	if( retval < 0 )
	{
		setIberr( EDVR );
		return -1;
	}

	setIberr( old_board_index );
	return 0;
}

int ibbna( int ud, char *board_name )
{
	ibConf_t *conf;
	int retval, result=0;
	int find_index;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

	if( ( find_index = ibGetIndexFromName( board_name ) ) < 0 )
	{
		setIberr( EARG );
		return func_exit( ud, 1 );
	}
	pthread_mutex_lock( m_lock[conf->settings.board] );
	retval = _ibbna( conf, find_index );
	if( retval < 0 )
	{
		result = func_exit( ud, 1 );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}

	result = func_exit( ud, 0 );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}
