/***************************************************************************
 ibWrt.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibWrt.c of the Linux GPIB Package driver 
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
#include <sys/ioctl.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

//extern pthread_mutex_t* m_lock[GPIB_MAX_NUM_BOARDS];

int find_eos( const uint8_t *buffer, size_t length, int eos, int eos_flags )
{
	unsigned int i;
	unsigned int compare_mask;

	if( eos_flags & BIN ) compare_mask = 0xff;
	else compare_mask = 0x7f;

	for( i = 0; i < length; i++ )
	{
		if( ( buffer[i] & compare_mask ) == ( eos & compare_mask ) )
		return i+1;
	}
	return -1;
}

int send_data(ibConf_t *conf, const void *buffer, size_t count, int send_eoi, size_t *bytes_written)
{
	ibBoard_t *board;
	read_write_ioctl_t write_cmd;
	int retval;
	
	board = interfaceBoard( conf );

//	set_timeout( board, conf->settings.usec_timeout );

	write_cmd.buffer = (void*) buffer;
	write_cmd.count = count;
	write_cmd.end = send_eoi;
	write_cmd.handle = conf->handle;
	if(conf->settings.usec_timeout)
		write_cmd.usec_timeout = (conf->settings.usec_timeout>=LINETIMEOUT)? (conf->settings.usec_timeout+ADDTIMEOUT):conf->settings.usec_timeout;
	else
		write_cmd.usec_timeout = 0;
    //yuan add 10/23/06
	board->io_in_progress = 1;
//	printf("ibwrt_tmo: %d\n", write_cmd.usec_timeout);
	retval = ioctl( board->fileno, IBWRT, &write_cmd);
//	printf("send_data: %d\n", retval);
	if(retval < 0)
	{
		switch( errno )
		{
			case ETIMEDOUT:
				conf->timed_out = 1;
				setIberr( EABO );
				break;
			case EINTR:
				setIberr( EABO );
				break;
			case EIO:
				setIberr( ENOL );
				break;
			case EFAULT:
				write_cmd.count = 0;
				//fall-through
			default:
				setIberr( EDVR );
				setIbcnt( errno );
				break;
		}
    //yuan add 10/23/06
	board->io_in_progress = 0;
		return -1;
	}
	*bytes_written = write_cmd.count;
//	conf->end = send_eoi && (*bytes_written == count);
	if(retval < 0) return retval;
	board->io_in_progress = 0;
	return count;
	//return 0;
}

int write_data(ibConf_t *conf, const void *buffer, size_t count,
	int force_eoi, size_t *bytes_written)
{
	int eoi_on_eos;
	int eos_found = 0;
	int send_eoi;
	unsigned long block_size=0;
	int retval;

	eoi_on_eos = conf->settings.eos_flags & XEOS;

	block_size = count;

	if( eoi_on_eos )
	{
		retval = find_eos( buffer, count, conf->settings.eos, conf->settings.eos_flags );
		if( retval < 0 ) eos_found = 0;
		else
		{
			block_size = retval;
			eos_found = 1;
		}
	}

	send_eoi = force_eoi || ( eoi_on_eos && eos_found );
	if(send_data(conf, buffer, block_size, send_eoi, bytes_written) < 0)
	{
		return -1;
	}
	return 0;
}

int _ibwrt( ibConf_t *conf,
	const uint8_t *buffer, size_t count, size_t *bytes_written)
{
	ibBoard_t *board;
	size_t block_size=0;
	int retval;
	
	*bytes_written = 0;
	board = interfaceBoard( conf );
	if(conf->async.in_progress) {
		conf->async.opstart = 1;
		//SetEvent( conf->async.startE);
	}

	if( conf->is_interface == 0 )
	{
		if(conf->async.in_progress && conf->async.abort) {
			conf->async.abort = 0;
			conf->async.opend = 1;
//	printf("ibwrt_011 error\n");
			return -1;
		}
		// set up addressing
		if( send_setup( conf ) < 0 )
		{
			conf->async.opend = 1;
//	printf("ibwrt_012 error\n");
			return -1;
		}
	}

	while( count )
	{
		if(conf->async.in_progress && conf->async.abort) {
			conf->async.abort = 0;
			conf->async.opend = 1;
//	printf("ibwrt_01 error\n");
			return -1;
		}
			//printf("sd0\n");

		retval = write_data( conf, buffer, count, conf->settings.send_eoi, &block_size);
		*bytes_written += block_size;
		if(retval < 0)
		{
			conf->async.opend = 1;
//	printf("ibwrt_02 error\n");			
			return -1;
		}
		count -= block_size;
		buffer += block_size;
	}
conf->async.opend = 1;	
	return 0;
}

int ibwrt( int ud, const void *rd, size_t cnt )
{
	ibConf_t *conf;
	size_t count;
	int retval, result=0;
//	struct sembuf   semBuf;

	conf = func_init( ud );
	if( conf == NULL )
	{
		return func_exit( ud, 1 );
	}

/*{

      semBuf.sem_num = 0;
       semBuf.sem_op = -1;
       semBuf.sem_flg = SEM_UNDO;
       if (semop(shr_sem[0], &semBuf, 1) != 0)
                printf(" RC_LOCK_ERROR\n");
}*/
//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[conf->settings.board] );
	if( conf->async.in_progress && (!(conf->async.ibsta & CMPL)))
	{
		{			
		    setIberr( EOIP );
		    result = _func_exit( ud, 1, 0, 0, 0, 0, 1 );
		    //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		    pthread_mutex_unlock( m_lock[conf->settings.board] );
  		    return result; 
		}
	}
	setIbcnt( 0 );
	setIbsta( 0 );
	conf->end = 0;

	retval = _ibwrt(conf, rd, cnt, &count);
	if(retval < 0)
	{
		if(ThreadIberr() != EDVR) setIbcnt(count);
		result = func_exit( ud, 1 );
		//pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}
	setIbcnt(count);
	result = _func_exit( ud, 0, 0, 0, DCAS, 0, 0 );
        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
/*semBuf.sem_num = 0;
       semBuf.sem_op  = 1;
       semBuf.sem_flg = SEM_UNDO;

       if (semop(shr_sem[0], &semBuf, 1) != 0)
           printf(" RC_UNLOCK_ERROR\n");*/
	return result;
}

int ibwrta( int ud, const void *buffer, size_t cnt )
{
	ibConf_t *conf;
	int retval, result=0;

	conf = _func_init( ud, 1, 0 );
	if( conf == NULL )
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );

//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[conf->settings.board] );
        setIbcnt( 0 );
    	setIbsta( 0 );
	conf->end = 0;
	retval = aio_setup( ud, conf, GPIB_AIO_WRITE,
		(void*)buffer, cnt );
	if( retval < 0 )
	{
	   result = _func_exit( ud, 1, 0, 0, 0, 0, 1 );
	   pthread_mutex_unlock( m_lock[conf->settings.board] );
	   //printf("ibwrta failed: %d\n", retval);
 	   return result;
	}
	result = _func_exit( ud, 0, 0, 0, CMPL, 0, 1 );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
//        pthread_mutex_unlock( &semap[conf->settings.board]->lock );
//printf("leave ibwrta\n");
return result;
}

int _ibwrtf( ibConf_t *conf, const char *file_path, size_t *bytes_written)
{
	ibBoard_t *board;
	size_t count;
	size_t block_size;
	int retval;
	FILE *data_file;
	struct stat file_stats;
	uint8_t buffer[ 0x4000 ];

	*bytes_written = 0;
	board = interfaceBoard( conf );

	data_file = fopen( file_path, "r" );
	if( data_file == NULL )
	{
		setIberr( EFSO );
		setIbcnt( errno );
		return -1;
	}

	retval = fstat( fileno( data_file ), &file_stats );
	if( retval < 0 )
	{
		setIberr( EFSO );
		setIbcnt( errno );
		return -1;
	}

	count = file_stats.st_size;

	if( conf->is_interface == 0 )
	{
		// set up addressing
		if( send_setup( conf ) < 0 )
		{
			return -1;
		}
	}

//	set_timeout( board, conf->settings.usec_timeout );

	while( count )
	{
		size_t fread_count;
		int send_eoi;
		size_t buffer_offset = 0;
		
		fread_count = fread( buffer, 1, sizeof( buffer ), data_file );
		if( fread_count == 0 )
		{
			setIberr( EFSO );
			setIbcnt( errno );
			return -1;
		}
		while(buffer_offset < fread_count)
		{
			send_eoi = conf->settings.send_eoi && (count == fread_count - buffer_offset);
			retval = write_data(conf, buffer + buffer_offset,
				fread_count - buffer_offset, send_eoi, &block_size);
			count -= block_size;
			buffer_offset += block_size;
			*bytes_written += block_size;
			if(retval < 0)
			{
				return -1;
			}
		}
	}
	return 0;
}

int ibwrtf( int ud, const char *file_path )
{
	ibConf_t *conf;
	size_t count;
	int retval, result=0;
	
	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[conf->settings.board] );
	conf->end = 0;
	setIbcnt( 0 );
	setIbsta( 0 );
	
	retval = _ibwrtf(conf, file_path, &count);
	if(retval < 0)
	{
		if(ThreadIberr() != EDVR) setIbcnt(count);
		result = func_exit( ud, 1 );
	        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		return result;
	}
	setIbcnt( count );
	result = _func_exit( ud, 0, 0, 0, DCAS, 0, 0 );
//        pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}

int _SendDataBytes( ibConf_t *conf, const void *buffer,
	size_t count, int eotmode)
{
	int retval;
	size_t num_bytes;
	size_t bytes_written = 0;
	
	if( conf->is_interface == 0 )
	{
		setIberr(EARG);
		return -1;
	}

	switch( eotmode )
	{
		case DABend:
		case NLend:
		case NULLend:
			break;
		default:
			setIberr( EARG );
			return -1;
			break;
	}
	retval = send_data( conf, buffer, count, eotmode == DABend, &num_bytes);
	bytes_written += num_bytes;
	if( retval < 0 )
	{
		setIbcnt(bytes_written);
		return retval;
	}
	if( eotmode == NLend )
	{
		retval = send_data( conf, "\n", 1, 1, &num_bytes);
		bytes_written += num_bytes;
		if( retval < 0 )
		{
			setIbcnt(bytes_written);
			return retval;
		}
	}
	setIbcnt(bytes_written);
	return 0;
}

void SendDataBytes( int boardID, const void *buffer,
	size_t count, int eotmode )
{
	ibConf_t *conf;
	int retval;

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}
	//pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[boardID] );
	setIbcnt( 0 );	
	setIbsta( 0 );
	conf->end = 0;
	retval = _SendDataBytes( conf, buffer, count, eotmode );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
	        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	_func_exit( boardID, 0, 0, 0, DCAS, 0, 0 );
        //pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}

int _SendList( ibConf_t *conf, const Addr4882_t addressList[],
	const void *buffer, size_t count, int eotmode )
{
	ibBoard_t *board;
	int retval;

	if( check_addressList( addressList ) == 0 ||
		numofAddresses( addressList ) == 0 )
	{
		setIberr(EARG);
		return -1;
	}

	if( conf->is_interface == 0 )
	{
		setIberr(EARG);
		return -1;
	}

	board = interfaceBoard( conf );
	//yuan add 01/13/12
	if(board->pci_bus>=0)
	{

	 if( check_cic( board ) == 0 )
	 { 
		setIberr( ECIC );
		return -1;
	 }
	}
	retval = _SendSetup( conf, addressList );
	if( retval < 0 ) return retval;

	retval = _SendDataBytes( conf, buffer, count, eotmode );
	if( retval < 0 ) return retval;

	return 0;
}

void SendList( int boardID, const Addr4882_t addressList[],
	const void *buffer, size_t count, int eotmode )
{
	ibConf_t *conf;
	int retval;

	conf = func_init( boardID );
	if( conf == NULL )
	{
		func_exit( boardID, 1 );
		return;
	}

//	pthread_mutex_lock( &semap[conf->settings.board]->lock );
	pthread_mutex_lock( m_lock[boardID] );

	setIbcnt( 0 );	
	setIbsta( 0 );
	conf->end = 0;
	retval = _SendList( conf, addressList, buffer, count, eotmode );
	if( retval < 0 )
	{
		func_exit( boardID, 1 );
//	        pthread_mutex_unlock( &semap[conf->settings.board]->lock );
		pthread_mutex_unlock( m_lock[boardID] );
		return;
	}
	_func_exit( boardID, 0, 0, 0, DCAS, 0, 0 );
//        pthread_mutex_unlock( &semap[conf->settings.board]->lock );
	pthread_mutex_unlock( m_lock[boardID] );
	return;
}

void Send( int boardID, Addr4882_t address, const void *buffer, size_t count,
	int eotmode )
{
	Addr4882_t addressList[ 2 ];

	addressList[ 0 ] = address;
	addressList[ 1 ] = NOADDR;

	SendList( boardID, addressList, buffer, count, eotmode );
}


