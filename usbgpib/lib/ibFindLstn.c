/***************************************************************************
 ibFindLstn.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibFindLstn.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2002,2003 by Frank Mori Hess <fmhess@users.sourceforge.net>
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
#include <unistd.h>
#include <stdlib.h>

int _find_listener( ibConf_t *conf, const Addr4882_t addressList[] )
{
	uint8_t *cmd;
	int i, j;
	short line_status;
	int retval;
	int board_pad=0, board_sad=0;
   	//char devStr[100];

	if( addressList == NULL )
		return 0;
	if( check_addressList( addressList ) == 0 )
		return -1;

	cmd = malloc( 16 + 2 * numofAddresses( addressList ) );
	if( cmd == NULL )
	{
		setIberr( EDVR );
		setIbcnt( ENOMEM );
		return -1;
	}

	j = 0;
	cmd[ j++ ] = UNL;
	for( i = 0; i < numofAddresses( addressList ); i++ )
	{
		int pad, sad;

		pad = _getPAD( addressList[ i ] );
		sad = _getSAD( addressList[ i ] );
		cmd[ j++ ] = MLA( pad );
		if( sad >= 0 )
			cmd[ j++ ] = MSA( sad );
	}
	/* controller's talk address */
	if( ask_pad( interfaceBoard( conf ), (unsigned int *) &board_pad ) < 0 ) return 0;
	    cmd[ j++ ] = MTA( board_pad );
	if( ask_sad( interfaceBoard( conf ), &board_sad ) < 0 ) return 0;
	if( board_sad >= 0 )
		cmd[ j++ ] = MSA( board_sad );

	retval = _ibcmd( conf, cmd, j );

	free( cmd );
	cmd = NULL;

	if( retval < 0 ) return retval;

	retval = _ibgts( conf, 0 );
	if( retval < 0 ) return -1;

	usleep( 1500 );

	if( conf->is_interface == 0 )
	{
		retval = _iblines( ibConfigs[ interfaceBoard( conf )->ud], &line_status );
	} else
		retval = _iblines( conf, &line_status );

//	retval = _iblines( conf, &line_status );
    	retval = _ibcac( conf, 0 );
	if( retval < 0 ) return retval;

	if( ( line_status & ValidNDAC ) &&
		( line_status & BusNDAC ) )
	{
		return 1;
	}

	return 0;
}

int _findlstn_with_sec( ibConf_t *conf, unsigned int pad )
{
	Addr4882_t testAddress[ 32 ];
	int j;

	for( j = 0; j <= gpib_addr_max; j++ )
		testAddress[ j ] = _mkAddr( pad, j );
	testAddress[ j ] = NOADDR;
	return _find_listener( conf, testAddress );
}

void FindLstn( int boardID, const Addr4882_t padList[],
	Addr4882_t resultList[], size_t maxNumResults )
{
	int i;
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;
	int resultIndex;
	short line_status;
        int board_pad=0, board_sad=0;

	setIbcnt( 0 );

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}

	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		func_exit( boardID, 1 );
		return;
	}

	board = interfaceBoard( conf );

	retval = _iblines( conf, &line_status );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
		return;
	}
	if( ( line_status & ValidNDAC ) == 0 )
	{
		setIberr( ECAP );
		func_exit( boardID, 1 );
		return;
	}

	//resultIndex = 0;
	ask_pad( board, (unsigned int *) &board_pad );
	ask_sad( board, &board_sad );
	pthread_mutex_lock( m_lock[boardID] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	resultIndex = 0;
	for( i = 0; i < numofAddresses( padList ); i++ )
	{
		Addr4882_t pad;
		Addr4882_t testAddress[ 2 ];

		pad = GetPAD( padList[ i ] );
		testAddress[ 0 ] = pad;
		testAddress[ 1 ] = NOADDR;
//yuan add 08/16/06
	 if(pad == board_pad)
		   retval = 1;
     else 
		retval = _find_listener( conf, testAddress );
		if( retval < 0 )
		{
			func_exit( boardID, 1 );
//			pthread_mutex_unlock( &semap[conf->settings.board]->lock );
			pthread_mutex_unlock( m_lock[boardID] );
			return;
		}
		if( retval > 0 )
		{
			if( resultIndex >= maxNumResults )
			{
				setIberr( ETAB );
				func_exit( boardID, 1 );
				pthread_mutex_unlock( m_lock[boardID] );
//				pthread_mutex_unlock( &semap[conf->settings.board]->lock );
				return;
			}
			resultList[ resultIndex++ ] = testAddress[ 0 ];
			setIbcnt( resultIndex );
		}else
		{
			retval = _findlstn_with_sec( conf, pad );
			if( retval < 0 )
			{
				func_exit( boardID, 1 );
				pthread_mutex_unlock( m_lock[boardID] );
//				pthread_mutex_unlock( &semap[conf->settings.board]->lock );
				return;
			}
			if( retval > 0 )
			{
				int j;
				for( j = 0; j <= gpib_addr_max; j++ )
				{
					testAddress[ 0 ] = _mkAddr( pad, j );
					testAddress[ 1 ] = NOADDR;
					retval = _find_listener( conf, testAddress );
					if( retval < 0 )
					{
						func_exit( boardID, 1 );
						pthread_mutex_unlock( m_lock[boardID] );
						//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
						return;
					}
					if( retval >= 1 )
					{
						if( resultIndex >= maxNumResults )
						{
							setIberr( ETAB );
							func_exit( boardID, 1 );
							pthread_mutex_unlock( m_lock[boardID] );
							//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
							return;
						}
						resultList[ resultIndex++ ] = testAddress[ 0 ];
						setIbcnt( resultIndex );
					}
				}
			}
		}
	}
	func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
//	pthread_mutex_unlock( &semap[conf->settings.board]->lock );
} /* FindLstn */

int ibln( int ud, int pad, int sad, short *found_listener )
{
	ibConf_t *conf;
	Addr4882_t addressList[ 2 ];	
	int retval, result=0;
	ibBoard_t *board;
	int board_pad=0, board_sad=0; 

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	board = interfaceBoard( conf );
	ask_pad( board, (unsigned int *) &board_pad );
	ask_sad( board, &board_sad );
	if(pad == board_pad) {
	  if(sad) {
		  if( board_sad >= 0 ) {
		   board_sad |= MSA( board_sad );
		   if(sad == board_sad) {
			*found_listener = 1;
			return func_exit( ud, 0 );
		   }
		  }
	  } else {
			*found_listener = 1;
			return func_exit( ud, 0 );
			//return func_exit( ud, 0 );
	  }
	}
	pthread_mutex_lock( m_lock[conf->settings.board] );
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	switch( sad )
	{
	case ALL_SAD:
		retval = _findlstn_with_sec( conf, pad );
		break;
	case NO_SAD:
	default:
		addressList[ 0 ] = MakeAddr( pad, sad );
		addressList[ 1 ] = NOADDR;
		retval = _find_listener( conf, addressList );
		break;
	}
	if( retval < 0 ) {
		result = func_exit( ud, 1 );
	   	//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}

	*found_listener = retval;

	result = func_exit( ud, 0 );
   	//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}
