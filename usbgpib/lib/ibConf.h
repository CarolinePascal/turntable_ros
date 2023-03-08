/***************************************************************************
 ibConf.h
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibConf.h of the Linux GPIB Package driver 
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
 
#ifndef _IBCONF_H
#define _IBCONF_H

#include <unistd.h>
#include <sys/types.h>
#include <pthread.h>

//yuan 09/22/05 add for time interval buffer
#define ADDTIMEOUT	1000000
#define LINETIMEOUT	3000000
/*---------------------------------------------------------------------- */

typedef int (* GpibNotifyCallback_t)(int, int, int, int, void *);

struct async_operation
{
	pthread_t thread;	
	pthread_mutex_t lock;
	pthread_mutex_t join_lock;
	uint8_t *buffer;
	volatile int buffer_length;
	volatile int iberr;
	volatile int ibsta;
	volatile int ibcntl;
	volatile short in_progress;
	volatile short abort;
	GpibNotifyCallback_t callback;
	void * cbRefData;
    //HANDLE startE;
    //HANDLE abortE;
	volatile unsigned long opstart:1;
	volatile unsigned long opend:1;
};

//yuan add 01/23/06
struct notify_operation
{
	int ud;
	pthread_t thread;	
	pthread_mutex_t lock;
	pthread_mutex_t join_lock;
	//unsigned char *buffer;
	//volatile long buffer_length;
	volatile int iberr;
	volatile int ibsta;
	volatile int ibcntl;
	volatile int mask;
	volatile short abort;
	GpibNotifyCallback_t callback;
	void * cbRefData;
};

typedef struct
{
	int pad;	/* primary address */
	int sad;	/* secodnary address (negative for disables) */
	int board;	/* board index */
	unsigned int usec_timeout;
	unsigned int spoll_usec_timeout;
	unsigned int ppoll_usec_timeout;
	char eos;                           /* eos */
	int eos_flags;
	int ppoll_config;	/* parallel poll configuration */
	int bus_timming;	/* t1 valye */
	unsigned send_eoi : 1;	/* assert EOI at end of writes */
	unsigned local_lockout : 1;	
	unsigned local_ppc : 1;	
	unsigned readdr : 1;
}descriptor_settings_t;

typedef struct ibConfStruct
{
	int handle;
	char name[100];		/* name of the interface */
	descriptor_settings_t defaults;	/* default settings */
	descriptor_settings_t settings;	/* software settings */
	char init_string[100];               
	char tmp_string[10];               
	int end_f;                         
	int handle_0;                         
	int usec_timeout;                         /* timeout interval    */
	int end_timeout;                         
	int flags;                         
	struct async_operation async;	
	struct 	notify_operation asyno;
	volatile unsigned end : 1;	
	volatile unsigned is_interface : 1;	
	volatile unsigned board_is_open : 1;
	volatile unsigned has_lock : 1;
	volatile unsigned timed_out : 1;	/* io operation timed out happened */	
} ibConf_t;

extern int ibnotify (int ud, int mask, GpibNotifyCallback_t Callback, void *RefData);
/*---------------------------------------------------------------------- */

typedef struct ibBoardStruct {
	char board_type[100];	/* name of interface */
	unsigned int base[6];  /* io base address */
	unsigned int irq;
	unsigned int dma;
	int pci_bus;
	int pci_slot;
	int ud;
	int fileno;             /* device file descriptor           */
	char device[100];	
	unsigned long hIntEvent[8];
	unsigned int open_count;	/* reference count */
	unsigned is_system_controller : 1;	/* system controller or not */
	unsigned use_event_queue : 1;	
	unsigned autospoll : 1; /* auto spolling */
	unsigned assert_ifc : 1;
	unsigned defautospoll : 1; 
	//yuan add 10/20/06
	unsigned io_in_progress: 1;
	//yuan add 04/12/07
	unsigned thread_open: 1;
	//yuan add 05/26/08
	unsigned rem_sta: 1;
} ibBoard_t;

#endif	/* _IBCONF_H */

















