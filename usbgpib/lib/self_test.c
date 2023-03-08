/***************************************************************************
 self_test.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/self_test.c of the Linux GPIB Package driver 
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
#include <stdlib.h>

int _TestSys( ibConf_t *conf, const Addr4882_t * addressList, short *resultList )
{
	unsigned int failure_count = 0;
	ibBoard_t *board;
	int retval;
	int i;

	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		return -1;
	}

	board = interfaceBoard( conf );
	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return -1;
	}

	if( check_addressList( addressList ) == 0 )
	{
		setIberr( EARG );
		return -1;
	}

	if( numofAddresses( addressList ) == 0 )
	{
		setIberr( EARG );
		return -1;
	}

	retval = _SendList( conf, addressList, "*TST?", 5, NLend );
	if( retval < 0 ) return retval;

	for( i = 0; i < numofAddresses( addressList ); i++ )
	{
		char reply[ 16 ];

		retval = _Receive( conf, addressList[ i ], reply,
			sizeof( reply ) - 1, STOPend );
		if( retval < 0 )
			return -1;

		reply[ ThreadIbcnt() ] = 0;
		resultList[ i ] = strtol( reply, NULL, 0 );

		if( resultList[ i ] ) failure_count++;
	}

	setIbcnt( failure_count );

	return 0;
}

int _ResetSys( ibConf_t *conf, const Addr4882_t addressList[] )
{
	ibBoard_t *board;
	int retval;

	board = interfaceBoard( conf );

	if( check_addressList( addressList ) == 0 )
	{
		setIberr( EARG );
		return -1;
	}

	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		return -1;
	}

	if( check_sc( board ) == 0 )
	{
		setIberr( ESAC );
		return -1;
	}
//yuan remove 01/12/12
/**
	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return -1;
	}
**/
	retval = remote_enable( board, 1 );
	if( retval < 0 ) return retval;

	retval = _ibsic( conf );
	if( retval < 0 ) return retval;

	retval = _DevClearList( conf, NULL );
	if( retval < 0 ) return retval;

	retval = _SendList( conf, addressList, "*RST", 4, NLend );
	if( retval < 0 ) return retval;

	return 0;
}

void ResetSys( int boardID, const Addr4882_t addressList[] )
{
	ibConf_t *conf;
	int retval;

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	pthread_mutex_lock( m_lock[boardID]);
	retval = _ResetSys( conf, addressList );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID]);
		return;
	}
	func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID]);
	return;
}

void TestSys( int boardID, const Addr4882_t * addrlist, short *results )
{
	ibConf_t *conf;
	int retval;

	if(results)
		*results = 0;
	setIbcnt( 0 );
	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	//pthread_mutex_lock( &semap[boardID]->lock );
	pthread_mutex_lock( m_lock[boardID] );
	setIbsta( 0 );
	retval = _TestSys( conf, addrlist, results );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
		//pthread_mutex_unlock( &semap[boardID]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	func_exit( boardID, 0 );
//	pthread_mutex_unlock( &semap[boardID]->lock );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}
