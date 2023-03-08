/***************************************************************************
 ibCmd.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibCmd.c of the Linux GPIB Package driver 
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
#include <sys/ioctl.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

ssize_t _ibcmd_TA ( ibConf_t *conf, const uint8_t *buffer, size_t count, Addr4882_t address);
ssize_t _ibcmd_rw( ibConf_t *conf, const uint8_t *buffer, size_t count, short rw );

int ibcmd(int ud, const void *cmd_buffer, size_t cnt)
{
	ibConf_t *conf;
	ssize_t count;
	int result=0;

	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );

	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		result = func_exit( ud, 1 );
		return result;
	}
	pthread_mutex_lock( m_lock[conf->settings.board] );

	setIbcnt( 0 );
	setIbsta( 0 );

	count = _ibcmd( conf, cmd_buffer, cnt);
	if(count < 0)
	{
		result = func_exit( ud, 1);
	        pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}

	if(count != cnt)
	{
		result = func_exit( ud, 1);
	        pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}
	setIbcnt( cnt );
	result = func_exit( ud, 0 );
        pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}

int ibcmda( int ud, const void *cmd_buffer, size_t cnt )
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;

	conf = _func_init( ud, 1, 0 );
	if( conf == NULL )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );

	if( conf->is_interface == 0 )
	{
		setIberr( EARG );
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );
	}

 	board = interfaceBoard( conf );
	//yuan add 01/12/12
	if(board->pci_bus>=0)
	{
	 if( check_cic( board ) == 0 )
	 {
		setIberr( ECIC );
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );
	 }
        }

	retval = aio_setup( ud, conf, GPIB_AIO_COMMAND,
		(void*)cmd_buffer, cnt );
	if( retval < 0 )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );

	return _func_exit( ud, 0, 0, 0, CMPL, 0, 1 );
}

ssize_t _ibcmd( ibConf_t *conf, const uint8_t *buffer, size_t count)
{
	read_write_ioctl_t cmd;
	int retval;
	ibBoard_t *board;

	memset(&cmd, 0, sizeof(cmd));
	board = interfaceBoard( conf );
	 //yuan add 09/25/06
	if(conf->async.in_progress) { //also check if it is a system controller???
		conf->async.opstart = 1;
//		SetEvent( conf->async.startE);
	}
	//yuan add 01/12/12
	if(board->pci_bus>=0)
	{
	 if( check_cic( board ) == 0 )
	 {
		//setIberr( ECIC );
		//return -1;
		//yuan add 01/12/12
		if(board->is_system_controller) 
		{
			if(assert_ifc( board, 100 ))
			{
				setIberr( ECIC );
				return -1;
			}
		} else
                {
			setIberr( ECIC );
			return -1;
		}
	   	board->assert_ifc = 1;
	 }
        }
	if(conf->async.in_progress && conf->async.abort) {
		conf->async.abort = 0;
			return -1;
	}
	{
	 unsigned long vi=0;
	 signed char addr = 0; 
     for(vi=0;vi<count;vi++) {
		//YUAN ADD 08/22/06
		 if(buffer[vi] == SPE) {
		 cmd.end |= (4<<16);
		     //sprintf(devStr, "%s %x %x",_ibcmd",buffer[vi], vi); 
		    //MessageBox(NULL, devStr, "gpib32", MB_SYSTEMMODAL|MB_OK|MB_ICONEXCLAMATION);

		 }
	   else if(buffer[vi] == SPD)
		 cmd.end |= (8<<16);
	   else {
		addr = buffer[vi] - 0x40;
		if((addr>=0) && (addr<=0x1f)) {
		//	board->hIntEvent[7] = (ULONG) addr;
			cmd.end |= (2<<16);			
		}
		//YUAN ADD 10/07/05
		//addr = buffer[vi] - 0x20;
		//if((addr>=0) && (addr<=0x1f)){
		//	cmd.end |= (2<<16);
		//}
	   }
	 }
	}
	cmd.buffer = (void*)buffer;
	cmd.count = count;
	cmd.handle = conf->handle;
	//cmd.end = 0;
	if(conf->settings.usec_timeout)
		cmd.usec_timeout = (conf->settings.usec_timeout>=LINETIMEOUT)? (conf->settings.usec_timeout+ADDTIMEOUT):conf->settings.usec_timeout;	
	//set_timeout( board, conf->settings.usec_timeout);
	board->io_in_progress = 1;
	retval = ioctl( board->fileno, IBCMD, &cmd );
	if( retval < 0 )
	{
		switch( errno )
		{
			case ETIMEDOUT:
				setIberr( EABO );//EBUS );
				conf->timed_out = 1;
				break;
			//case EBUS for device error 
			default:
				setIberr( EDVR );
				setIbcnt( errno );
				break;
		}
		//yuan add 10/23/06
		board->io_in_progress = 0;
		return -1;
	}
	//yuan add 10/23/06
	board->io_in_progress = 0;
	return cmd.count;
}

unsigned int board_send_setup( const ibBoard_t *board,
	const Addr4882_t addressList[], uint8_t *cmdString )
{
	unsigned int i, j;
	unsigned int board_pad;
	int board_sad;

	if( addressList == NULL )
	{
		fprintf(stderr, "addressList NULL for board_send_setup()\n");
		return 0;
	}
	if( check_addressList( addressList ) == 0 )
	{
		fprintf(stderr, "libgpib: bug! bad address list\n");
		return 0;
	}

	i = 0;
	// talk address of the controller */
	if(ask_pad(board, &board_pad) < 0) return 0;
	cmdString[i++] = MTA(board_pad);
	if(ask_sad(board, &board_sad) < 0) return 0;
	if(board_sad >= 0 )
		cmdString[i++] = MSA(board_sad);
	cmdString[ i++ ] = UNL;
	for( j = 0; j < numofAddresses( addressList ); j++ )
	{
		unsigned int pad;
		int sad;

		pad = _getPAD( addressList[ j ] );
		sad = _getSAD( addressList[ j ] );
		cmdString[ i++ ] = MLA( pad );
		if( sad >= 0)
			cmdString[ i++ ] = MSA( sad );
	}

	return i;
}

unsigned int create_sent_cmd_string( const ibConf_t *conf,
	uint8_t *cmdString )
{
	ibBoard_t *board;
	Addr4882_t addressList[ 2 ];

	board = interfaceBoard( conf );

	addressList[ 0 ] = _mkAddr( conf->settings.pad, conf->settings.sad );
	addressList[ 1 ] = NOADDR;

	return board_send_setup( board, addressList, cmdString );
}

int send_setup( ibConf_t *conf )
{
	uint8_t cmdString[8];
	int retval;

	retval = create_sent_cmd_string( conf, cmdString );

	//if( _ibcmd( conf, cmdString, retval ) < 0 )
	if( _ibcmd_rw( conf, cmdString, retval, 1 ) < 0 )
		return -1;

	return 0;
}

int _SendSetup( ibConf_t *conf, const Addr4882_t addressList[] )
{
	int i;
	ibBoard_t *board;
	uint8_t *cmd;
	int count;

	if( check_addressList( addressList ) == 0 ||
		numofAddresses( addressList ) == 0 )
	{
		setIberr( EARG );
		return -1;
	}

	if( conf->is_interface == 0 )
	{
		setIberr( EDVR );
		return -1;
	}

	board = interfaceBoard( conf );
	//yuan add 01/12/12
	if(board->pci_bus>=0)
	{
	if( check_cic( board ) == 0 )
	{
		setIberr( ECIC );
		return -1;
	}
        }
	cmd = malloc( 16 + 2 * numofAddresses( addressList ) );
	if( cmd == NULL )
	{
		setIberr( EDVR );
		setIbcnt( ENOMEM );
		return -1;
	}

	i = board_send_setup( board, addressList, cmd );

	//count = _ibcmd( conf, cmd, i );
	count = _ibcmd_rw( conf, cmd, i, 1 );

	free( cmd );
	cmd = NULL;

	if(count != i)
	{
		return -1;
	}

	return 0;
}

void SendSetup( int boardID, const Addr4882_t addressList[] )
{
	int retval;
	ibConf_t *conf;

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	pthread_mutex_lock( m_lock[boardID] );
	setIbcnt( 0 );	
	setIbsta( 0 );
	retval = _SendSetup( conf, addressList );
	if( retval < 0 )
	{
		if(retval == -2) {
			setIberr( EBUS );
			setIbsta(  CMPL | ERR );
			//no listener
			retval = -1;
		}
		func_exit( boardID, 1 );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	func_exit( boardID, 0 );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}

void SendCmds( int boardID, const void *buffer, size_t count )
{
	ibcmd( boardID, buffer, count );
}

ssize_t _ibcmd_TA ( ibConf_t *conf, const uint8_t *buffer, size_t count, Addr4882_t address)
{
	read_write_ioctl_t cmd;
	ibBoard_t *board;
	int retval = 0;
	
        memset((void *) &cmd, 0, sizeof(read_write_ioctl_t));

	board = interfaceBoard( conf );
	//yuan add 01/12/12
	if(board->pci_bus>=0)
	{
	if( check_cic( board ) == 0 )
	{
		//yuan add 09/05/08
		if(board->is_system_controller) {
			assert_ifc( board, 100 );
			board->assert_ifc = 1;
		} else {
			setIberr( ECIC );
			return -1;
		}
	}
	}
	if(conf->async.in_progress && conf->async.abort) {
		conf->async.abort = 0;
			return -1;
	}
	cmd.buffer = (void*)buffer;
	cmd.count = count;
	cmd.handle = conf->handle;
	cmd.end = (1<<16)|(address&0xffff);
	{
	 unsigned long vi=0;
	 signed char addr = 0; 
     for(vi=0;vi<count;vi++) {
		//YUAN ADD 08/22/06
		if(buffer[vi] == SPE)
		 cmd.end |= (4<<16);
	    else if(buffer[vi] == SPD)
		 cmd.end |= (8<<16);
	    else {
		 addr = buffer[vi] - 0x40;
		 if((addr>=0) && (addr<=0x1f)) {
			board->hIntEvent[7] = (unsigned long ) addr;
			cmd.end |= (2<<16);
		 }
		 //YUAN ADD 10/07/05
		 //addr = buffer[vi] - 0x20;
		 //if((addr>=0) && (addr<=0x1f)){
		//	cmd.end |= (2<<16);
		 //}
		}
	 }
	}
	//yuan add 10/20/06
	//if(!conf->settings.usec_timeout)
	//	cmd.usec_timeout = 0xffffffff;
	//else
	if(conf->settings.usec_timeout)
		cmd.usec_timeout = (conf->settings.usec_timeout>=LINETIMEOUT)? (conf->settings.usec_timeout+ADDTIMEOUT):conf->settings.usec_timeout;//conf->settings.usec_timeout;
	//...set_timeout( board, conf->settings.usec_timeout);
    //sprintf(devStr, "%s %s %d",_ibcmd",buffer, count); 
		    //MessageBox(NULL, "incmd_ta", "gpib32", MB_SYSTEMMODAL|MB_OK|MB_ICONEXCLAMATION);
    //yuan add 10/23/06
	board->io_in_progress = 1;

	retval = ioctl( board->fileno, IBCMD, &cmd );
	if( retval < 0 )
	{
		switch( errno )
		{
			case ETIMEDOUT:
				setIberr( EABO );//EBUS );
				conf->timed_out = 1;
				break;
			//case EBUS for device error 
			default:
				setIberr( EDVR );
				setIbcnt( errno );
				break;
		}
		//yuan add 10/23/06
		board->io_in_progress = 0;
		return -1;
	}
    //yuan add 10/23/06
	board->io_in_progress = 0;
	return cmd.count;
}

ssize_t _ibcmd_rw( ibConf_t *conf, const uint8_t *buffer, size_t count, short rw )
{
	read_write_ioctl_t cmd;
	int retval;
	ibBoard_t *board;
       //char devStr[1024];

   if(!rw) 
	   return _ibcmd( conf, buffer, count);

   memset((void *) &cmd, 0, sizeof(read_write_ioctl_t));
	board = interfaceBoard( conf );
//	printf("ibcmd00\n");
    //yuan add 09/25/06
	if(conf->async.in_progress) {
		conf->async.opstart = 1;
		//SetEvent( conf->async.startE);
	}
//printf("ibcmd000\n");
	//yuan add 01/12/12
	if(board->pci_bus>=0)
	{
	if( check_cic( board ) == 0 )
	{
		//yuan add 09/05/08
		if(board->is_system_controller) {
			assert_ifc( board, 100 );
			board->assert_ifc = 1;
		} else {
			setIberr( ECIC );
			return -1;
		}
	}
	}

	/**if( check_cic( board ) == 0 )
	{
		assert_ifc( board, 100 );
		board->assert_ifc = 1;

		//setIberr( ECIC );
		//return -1;
	}**/
//	printf("ibcmd0000\n");
	if(conf->async.in_progress && conf->async.abort) {
		conf->async.abort = 0;
			return -1;
	}
//	printf("ibcmd00000\n");
	//Sleep(10);
	{
	 unsigned long vi=0;
	 signed char addr = 0; 
     for(vi=0;vi<count;vi++) {
		//YUAN ADD 08/22/06
		 if(buffer[vi] == SPE) {
		 cmd.end |= (4<<16);
		     //sprintf(devStr, "%s %x %x","_ibcmd",buffer[vi], vi); 
		    //MessageBox(NULL, devStr, "gpib32", MB_SYSTEMMODAL|MB_OK|MB_ICONEXCLAMATION);

		 }
	   else if(buffer[vi] == SPD)
		 cmd.end |= (8<<16);
	   else {
		addr = buffer[vi] - 0x40;
		if((addr>=0) && (addr<=0x1f)) {
			board->hIntEvent[7] = (unsigned long) addr;
			cmd.end |= (2<<16);			
		}
		//YUAN ADD 10/07/05
		//addr = buffer[vi] - 0x20;
		//if((addr>=0) && (addr<=0x1f)){
		//	cmd.end |= (2<<16);
		//}
	   }
	 }
	}
	//yuan add 02/29/08
	cmd.end |= (1<<20);
	cmd.buffer = (void*)buffer;
	cmd.count = count;
	cmd.handle = conf->handle;
	//cmd.end = 0;
	//yuan add 10/20/06
	//if(!conf->settings.usec_timeout)
	//	cmd.usec_timeout = 0xffffffff;
	//else
	if(conf->settings.usec_timeout)
		cmd.usec_timeout = (conf->settings.usec_timeout>=LINETIMEOUT)? (conf->settings.usec_timeout+ADDTIMEOUT):conf->settings.usec_timeout;
	//...set_timeout( board, conf->settings.usec_timeout);
    //sprintf(devStr, "%s %s %d","_ibcmd",buffer, count); 
	//	    MessageBox(NULL, devStr, "gpib32", MB_SYSTEMMODAL|MB_OK|MB_ICONEXCLAMATION);
    //{
	//	char devStr[100];
	//	sprintf(devStr, "cmd: %d\n", cmd.usec_timeout);
	//	MessageBox(NULL, devStr, "gpib32", MB_SYSTEMMODAL|MB_OK|MB_ICONEXCLAMATION);
	//}
//yuan add 10/23/06
	//yuan add 08/20/06
//	printf("ibcmd0\n");
	board->io_in_progress = 1;

	retval = ioctl( board->fileno, IBCMD, &cmd );
	if( retval < 0 )
	{
//	printf("ibcmd1: %d %d\n", retval, errno);
		switch( errno )
		{
			case ETIMEDOUT:
				setIberr( EABO );//EBUS );
				conf->timed_out = 1;
				break;
			//case EBUS for device error 
			default:
				setIberr( EDVR );
				setIbcnt( errno );
				break;
		}
		//yuan add 10/23/06
		board->io_in_progress = 0;
		return -1;
	}
	//yuan add 10/23/06
	board->io_in_progress = 0;
//		printf("ibcmd1\n");

	return cmd.count;
}
