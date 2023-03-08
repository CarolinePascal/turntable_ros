/***************************************************************************
 async.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/async.c of the Linux GPIB Package driver 
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
#include <sys/ioctl.h>
#include <stdlib.h>
#include <pthread.h>

static void* aio_thread( void *varg );

struct gpib_aio_arg
{
	int ud;
	ibConf_t *conf;
	int gpib_aio_type;
};

void init_async_op( struct async_operation *async )
{
	async->buffer = NULL;
	async->buffer_length = 0;
	async->iberr = 0;
	async->ibsta = 0;
	async->ibcntl = 0;
	async->in_progress = 0;
	async->abort = 0;
	async->callback = NULL;
	async->cbRefData = NULL;
	async->thread = 0;
	async->opstart = 0;
	async->opend = 0;

	pthread_mutex_init( &async->lock, NULL );
	pthread_mutex_init( &async->join_lock, NULL );
}

void init_asyno_op( struct notify_operation *async )
{
	async->iberr = 0;
	async->ibsta = 0;
	async->ibcntl = 0;
	//async->in_progress = 0;
	async->abort = 0;
	async->mask = 0;
	async->callback = NULL;
	async->cbRefData = NULL;
	async->thread = 0;
	pthread_mutex_init( &async->lock, NULL );
	pthread_mutex_init( &async->join_lock, NULL );
}

int abort_aio( ibConf_t *conf)
{
	pad_ioctl_t pad_cmd;
	int retval =0;    
	pad_cmd.handle = conf->handle;
	pad_cmd.pad = 0;
	retval = ioctl( interfaceBoard( conf )->fileno, SW_IOC_ASYNC_STOP, &pad_cmd );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return -1;
	}
        return 0; 
}

int aio_setup( int ud, ibConf_t *conf, int gpib_aio_type,
	void *buffer, int cnt )
{
	int retval;
	struct gpib_aio_arg *arg;
	pthread_attr_t attributes;
   	ibBoard_t *board;
   
	arg = malloc( sizeof( *arg ) );
	if( arg == NULL )
	{
		setIberr( EDVR );
		setIbcnt( ENOMEM );
		return -1;
	}
	arg->ud = ud;
	arg->conf = conf;
	arg->gpib_aio_type = gpib_aio_type;

	pthread_mutex_lock( &conf->async.lock );
	if( conf->async.in_progress && (!(conf->async.ibsta & CMPL)))
	{
		//printf("async_in_process failed\n");
		free(arg);
		setIberr( EOIP );
		pthread_mutex_unlock( &conf->async.lock );		
		return -1;
	}
	board = interfaceBoard( conf );
    	setIbcnt( 0 );
	setIbsta( 0 );
	conf->async.ibsta = 0;
	conf->async.ibcntl = 0;
	conf->async.iberr = 0;
	conf->async.buffer = buffer;
	conf->async.buffer_length = cnt;
	conf->async.abort = 0;
	conf->async.in_progress = 0;
	conf->async.opstart = 0;
	conf->async.opend = 0;
	conf->async.in_progress = 1;
	pthread_attr_init( &attributes );
	pthread_attr_setstacksize( &attributes, 0x10000 );
	retval = pthread_create( &conf->async.thread,
		&attributes, aio_thread, arg );
	pthread_attr_destroy( &attributes );
	pthread_mutex_unlock( &conf->async.lock );
	if( retval )

	{
		free( arg ); arg = NULL;
		setIberr( EDVR );
		setIbcnt( retval );
		printf("pthread creation failed\n");
		return -1;
	}

	return 0;
}

static void* aio_thread( void *varg )
{
	size_t count;
	struct gpib_aio_arg arg;
	ibConf_t *conf;
	int retval;
	uint8_t end=0;

	arg = *((struct gpib_aio_arg*) varg);
	free( varg ); varg = NULL;

	conf = _func_init( arg.ud, 0, 1 );
	if( conf != arg.conf )
	{
		conf = arg.conf;
		pthread_mutex_lock( &conf->async.lock );
		conf->async.ibcntl = 0;
		conf->async.iberr = ThreadIberr();
		conf->async.ibsta = CMPL | ERR;
		pthread_mutex_unlock( &conf->async.lock );
		//pthread_mutex_unlock( m_lock[conf->settings.board] );
		return NULL;
	}	
	pthread_mutex_lock( m_lock[conf->settings.board] );
	//printf("aio_thread enter\n");
//	pthread_cleanup_push( cleanup_aio, &arg);//(ibConf_t *) &arg );
//	pthread_setcanceltype( PTHREAD_CANCEL_ASYNCHRONOUS, NULL );
//	pthread_setcancelstate( PTHREAD_CANCEL_ENABLE, NULL );

	switch( arg.gpib_aio_type )
	{
	case GPIB_AIO_COMMAND:
		count = retval = _ibcmd( conf, conf->async.buffer, conf->async.buffer_length );
		break;
	case GPIB_AIO_READ:
		retval = _ibrd( conf, conf->async.buffer, conf->async.buffer_length, &count, &end);
		break;
	case GPIB_AIO_WRITE:
		retval = _ibwrt(conf, conf->async.buffer, conf->async.buffer_length, &count);
		break;
	default:
		retval = -1;		
		break;
	}
//	pthread_setcancelstate( PTHREAD_CANCEL_DISABLE, NULL );
	pthread_mutex_lock( &conf->async.lock );
	if(retval < 0)
	{
		if(ThreadIberr() != EDVR)
			conf->async.ibcntl = count;
		else
			conf->async.ibcntl = ThreadIbcntl();
		conf->async.iberr = ThreadIberr();
		conf->async.ibsta = CMPL | ERR;
            	//printf("error %x %x\n", conf->async.ibsta, conf->async.iberr);
	}else
	{
		conf->async.ibcntl = count;
		conf->async.iberr = 0;
		if( conf->end || end) {
			conf->end = 1;
			conf->async.ibsta = END;			
            	//printf("end\n");
		}  
		conf->async.ibsta |= CMPL;
	}
	//yuna add 07/27/09
	if(conf->async.callback) 
	{
		int result;
		//printf("enter callback\n");
		pthread_mutex_unlock( &conf->async.lock );
		ibstatus( conf, 0, 0, 0 );
		pthread_mutex_lock( &conf->async.lock );
		sync_globals();	
		pthread_mutex_unlock( &conf->async.lock );
		//printf("enter callback 2\n");
		result = conf->async.callback(arg.ud, conf->async.ibsta, conf->async.iberr, conf->async.ibcntl, conf->async.cbRefData);
		conf->async.in_progress = 0;
		//printf("leave callback  res: %d\n", result);
		//yuan add 01/23/06
		if(!result) {
			//ibnotify (arg.ud, 0, NULL, NULL);		
			conf->async.callback = 0;
			conf->async.cbRefData = 0;
		}
		//printf("leave callback 2\n");	
		pthread_mutex_unlock( m_lock[conf->settings.board] );
		//	pthread_cleanup_pop( 1 );
		return NULL;		
	}
	pthread_mutex_unlock( &conf->async.lock );
	pthread_mutex_unlock( m_lock[conf->settings.board] );
//	pthread_cleanup_pop( 1 );
	return NULL;
}

int aio_thread_join( struct async_operation *async )
{
	int retval;

	pthread_mutex_lock( &async->join_lock );
	retval = pthread_join( async->thread, NULL );
	switch( retval )
	{
	case 0:
		break;
	case ESRCH:	//already been joined
		retval = 0;
		break;
	default:		
		setIberr( EDVR );
		setIbcnt( retval );
		break;
	}
	pthread_mutex_unlock( &async->join_lock );
	return retval;
}
