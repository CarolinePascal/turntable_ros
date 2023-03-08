/***************************************************************************
 globals.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/globals.c of the Linux GPIB Package driver 
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
#include <pthread.h>
#include <stdlib.h>

volatile int iberr = 0;
volatile int ibsta = 0;
volatile int ibcnt = 0;
volatile int ibcntl = 0;

static pthread_key_t ibsta_key;
static pthread_key_t iberr_key;
static pthread_key_t ibcntl_key;

static pthread_once_t global_keys_once = PTHREAD_ONCE_INIT;

static void ibsta_free( void *thread_ibsta )
{
	if( thread_ibsta )
	{
		free( thread_ibsta );
		thread_ibsta = NULL;
	}
}

static void iberr_free( void *thread_iberr )
{
	if( thread_iberr )
	{
		free( thread_iberr );
		thread_iberr = NULL;
	}
}

static void ibcntl_free( void *thread_ibcntl )
{
	if( thread_ibcntl )
	{
		free( thread_ibcntl );
		thread_ibcntl = NULL;
	}
}

static void global_keys_create(void)
{
	int retval;

	retval = pthread_key_create( &ibsta_key, ibsta_free );
	if( retval ) printf( "Thread specified Data creation failed for ibsta!\n" );
	retval = pthread_key_create( &iberr_key, iberr_free );
	if( retval ) printf( "Thread specified Data creation failed for iberr!\n" );
	retval = pthread_key_create( &ibcntl_key, ibcntl_free );
	if( retval ) printf( "Thread specified Data creation failed for ibcntl!\n" );
}

void globals_create( void )
{
	int *ibsta_p, *iberr_p, *ibcntl_p;

	pthread_once( &global_keys_once, global_keys_create );
	if( pthread_getspecific( ibsta_key ) == NULL )
	{
		ibsta_p = malloc( sizeof( int ) );
		if( ibsta_p == NULL )
		printf( "ibsta memory allocation failed!\n" );
		iberr_p = malloc( sizeof( int ) );
		if( iberr_p == NULL )
		printf( "iberr memory allocation failed!\n" );
		ibcntl_p = malloc( sizeof( int ) );
		if( ibcntl_p == NULL )
		printf( "ibcntl memory allocation failed!\n" );
		*ibsta_p = 0;
		*iberr_p = 0;
		*ibcntl_p = 0;
		pthread_setspecific( ibsta_key, ibsta_p );
		pthread_setspecific( iberr_key, iberr_p );
		pthread_setspecific( ibcntl_key, ibcntl_p );
	}
}

void setIbsta( int status )
{
	int *thread_ibsta;

	globals_create();
	thread_ibsta = pthread_getspecific( ibsta_key );
	if( thread_ibsta == NULL )
	{
		printf("Failed to set thread ibsta\n" );
		return;
	}
	*thread_ibsta = status;
}

void setIberr( int error )
{
	int *thread_iberr;
	globals_create();
	thread_iberr = pthread_getspecific( iberr_key );
	if( thread_iberr == NULL )
	{
		printf( "Failed to set thread iberr\n" );
		return;
	}
	*thread_iberr = error;
}

void setIbcnt( int count )
{
	int *thread_ibcntl;

	globals_create();
	thread_ibcntl = pthread_getspecific( ibcntl_key );
	if( thread_ibcntl == NULL )
	{
		printf( "Failed to set thread ibcntl\n" );
		return;
	}
	*thread_ibcntl = count;
}

int ThreadIbsta( void )
{
	int *thread_ibsta;

	globals_create();

	thread_ibsta = pthread_getspecific( ibsta_key );
	if( thread_ibsta == NULL )
	{
		printf( "Failed to get thread ibsta\n" );
		return ERR;
	}

	return *thread_ibsta;
}

int ThreadIberr( void )
{
	int *thread_iberr;

	globals_create();

	thread_iberr = pthread_getspecific( iberr_key );
	if( thread_iberr == NULL )
	{
		printf( "Failed to get thread iberr\n" );
		return EDVR;
	}

	return *thread_iberr;
}

int ThreadIbcnt( void )
{
	return ThreadIbcntl();
}

int ThreadIbcntl( void )
{
	int *thread_ibcntl;

	globals_create();
	thread_ibcntl = pthread_getspecific( ibcntl_key );
	if( thread_ibcntl == NULL )
	{
		printf( "Failed to get thread ibcntl\n" );
		return 0;
	}

	return *thread_ibcntl;
}

void sync_globals( void )
{
	ibsta = ThreadIbsta();
	iberr = ThreadIberr();
	ibcntl = ThreadIbcnt();
	ibcnt = ibcntl;
}
