/***************************************************************************
 ibutil.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibutil.c of the Linux GPIB Package driver 
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <assert.h>
#include <signal.h>

ibConf_t *ibConfigs[ GPIB_CONFIGS_LENGTH ] = {NULL};
ibConf_t ibFindConfigs[ FIND_CONFIGS_LENGTH ];
pthread_mutex_t* m_lock[GPIB_MAX_NUM_BOARDS];
int mshmid[GPIB_MAX_NUM_BOARDS];
static volatile int config_parsed;
unsigned short NumberOfCard;
int LoadConfiguration(void) ;
int setup_board_descriptors( void );
void adgpib_init(void);
int parse_board_t1_delay( int t1_delay );

void __attribute__((constructor)) app_init(void);
void __attribute__((destructor)) app_fini(void);

struct sigaction old_action;

void sigint_handler(int sig_no)
{
    //printf("CTRL-C pressed\n");
switch(sig_no)
{
case SIGINT: 
printf("CTRL-C pressed\n");
break;
case SIGSEGV: 
printf("SIGSEGV error\n");
break;
case SIGILL: 
printf("SIGILL error\n");
break;
case SIGFPE: 
printf("SIGFPE error\n");
break;
}
    app_fini();
    sigaction(SIGINT, &old_action, NULL);
    kill(0, SIGINT);
}


void __attribute__((constructor)) app_init(void)
{
int vi=0;
struct sigaction action;

///printf("app init\n");

//here to load adgpib.cfg
NumberOfCard=0;
config_parsed=0;
adgpib_init();
LoadConfiguration();
for (vi=0;vi<NumberOfCard;vi++)
{
char fileN[256];
sprintf(fileN, "ad%d", vi);
m_lock[vi] = 
semaphore_create(fileN, &mshmid[vi]);
if (m_lock[vi] == NULL)
{	
	printf("semphore[%d] create failed\n", vi);
}
//sprintf(fileN, "adg%d", vi);
 //semaphore_create_2(fileN, &shr_sem[vi]);
}
//printf("NumberOfCard %d\n", NumberOfCard);
setup_board_descriptors();

memset(&action, 0, sizeof(action));
action.sa_handler = &sigint_handler;
sigaction(SIGINT, &action, &old_action);
signal(SIGSEGV, sigint_handler);
signal(SIGILL, sigint_handler);
signal(SIGFPE, sigint_handler);
///printf("appinit finished\n");
}


void __attribute__((destructor)) app_fini(void)
{
int vi=0;
///printf("app end\n");
for (vi=0;vi<NumberOfCard;vi++)
{
semaphore_close(m_lock[vi], mshmid[vi]);
}
}

void adgpib_init(void)
{
int devcnt=0, i=0;
	        
	for( i = 0; i < FIND_CONFIGS_LENGTH; i++ ) {
		init_ibconf( &ibFindConfigs[ i ] );
        }
	for( i = 0; i < GPIB_MAX_NUM_BOARDS; i++ ) {
			init_ibboard( &ibBoard[ i ] );
	}
	for(devcnt=0; devcnt<GPIB_MAX_NUM_BOARDS;devcnt++)
	{ 
	ibBoard[devcnt].irq = 0;
	ibBoard[devcnt].pci_bus = -1;//0; //yuan modify 01/13/12
	ibBoard[devcnt].pci_slot = 0;
	ibBoard[devcnt].fileno = -1;
	ibBoard[devcnt].ud = -1;
	ibBoard[devcnt].assert_ifc = 0;
	//yuan add 04/12/07
	ibBoard[devcnt].thread_open = 0;
	//strcpy(ibBoard[devcnt].device, "gpib");
	sprintf(ibBoard[devcnt].board_type, "%s%d", "adgpib", devcnt);
	sprintf(ibBoard[devcnt].device, "%s%d", "/dev/adgpib", devcnt);
	ibBoard[devcnt].is_system_controller = 0;//1;
	ibFindConfigs[devcnt].handle = -1;
	sprintf(ibFindConfigs[devcnt].name, "%s%d", "gpib", devcnt);
	ibFindConfigs[devcnt].defaults.pad = 0;
	ibFindConfigs[devcnt].defaults.sad = -1;
	////ibFindConfigs[devcnt].defaults.board = devcnt;
	ibFindConfigs[devcnt].defaults.usec_timeout = 10000000;
	ibFindConfigs[devcnt].defaults.spoll_usec_timeout = 1000000;
	ibFindConfigs[devcnt].defaults.ppoll_usec_timeout = 2;
	ibFindConfigs[devcnt].defaults.eos = 0x0a;
	ibFindConfigs[devcnt].defaults.eos_flags = 0;//REOS;
	ibFindConfigs[devcnt].defaults.bus_timming = 2; //500ns
	ibFindConfigs[devcnt].init_string[0] = 0;
	ibFindConfigs[devcnt].is_interface = 1;
	ibFindConfigs[devcnt].timed_out = 0;
	}
}

int LoadConfiguration(void) 
{
	char	buf[256];
	FILE *conf_file;	
	int i=0, count=0, index;
	char *line;
        size_t len = 0;
        ssize_t read;
	char * m_line = NULL;
	unsigned long item=0;
	unsigned short cnt=0;

	//strcat(buf, "/etc/adgpib/adgpib.cfg");
	sprintf(buf, "%s", "/etc/adgpib/adgpib.cfg");
	conf_file=fopen(buf, "r");
	if(conf_file==0)
	{
		////printf("open failed %d\n", errno);
		return count;
	}

line = malloc(100);
m_line = malloc(100);
	

  while ((read=getline(&m_line, &len, conf_file)) != -1)
	{
		if((line = strstr(m_line, "number_of_gpib")))
		{
			line = strstr(line, "=");
			NumberOfCard = (unsigned short) atoi(++line);
			//printf("totalcard: %d\n", NumberOfCard);
			break;
		}		
	}	
        if(read == -1)
		return 0;
      while(feof(conf_file) == 0 ) 
      {
        while ((read=getline(&m_line, &len, conf_file)) != -1)
	{
		if((line = strstr(m_line, "[adgpib")))
		{
			line += 7;
			line[1]='\0';
			index = (unsigned short) atoi(line);
			//printf("index:%d\n", index);
			if(index>=GPIB_MAX_NUM_BOARDS)
				return 0;
			ibFindConfigs[index].defaults.board = index;
			break;
		}		
	}	
        if(read == -1)
	{
	
		//printf("damn: %d\n", read);
		//return 0;
		goto endfun;
	}
	item = 0;
        
	read=getline(&m_line, &len, conf_file);
	if((line = strchr(m_line, '=')) != NULL)
	{
		line += 1;
		ibBoard[index].is_system_controller = (unsigned short) atoi(line);
		//printf("sc:%d\n",ibBoard[index].is_system_controller);
//		break;
	}		
	read=getline(&m_line, &len, conf_file);
	if((line = strchr(m_line, '=')) != NULL)
	{
		line += 1;
		ibFindConfigs[index].defaults.pad = atoi(line);
		//printf("pa:%d\n", ibFindConfigs[index].defaults.pad);
//		break;
	}
	read=getline(&m_line, &len, conf_file);		
	if((line = strchr(m_line, '=')) != NULL)
	{
		line += 1;
		ibFindConfigs[index].defaults.sad = atoi(line);
		//printf("sa:%d\n", ibFindConfigs[index].defaults.sad);
//		break;
	}		
	read=getline(&m_line, &len, conf_file);
	if((line = strchr(m_line, '=')) != NULL)
	{
		line += 1;
		ibFindConfigs[index].defaults.bus_timming = parse_board_t1_delay(atoi(line));
		//printf("t1:%d\n", ibFindConfigs[index].defaults.bus_timming);
//		break;
	}		
	read=getline(&m_line, &len, conf_file);
	if((line = strchr(m_line, '=')) != NULL)
	{
		line += 1;
		ibFindConfigs[index].defaults.usec_timeout = atoi(line);
		//printf("t1:%d\n", ibFindConfigs[index].defaults.usec_timeout);
//		break;
	}		
	read=getline(&m_line, &len, conf_file);
	if((line = strchr(m_line, '=')) != NULL)
	{
		line += 1;
		ibBoard[index].defautospoll = atoi(line);
		//printf("autopoll:%d\n",ibBoard[index].defautospoll);
//		break;
	}
	cnt++;
	if(cnt>=NumberOfCard) break;
	}
endfun:	
//if(m_line)	
  // free(m_line);
   //free(line);
  //fclose( conf_file );
  for(i = 0; i < FIND_CONFIGS_LENGTH && ibFindConfigs[ i ].defaults.board >= 0; i++) {
	ibFindConfigs[ i ].settings = ibFindConfigs[ i ].defaults;
  }
  config_parsed = 1;
	return NumberOfCard;	
}
int add_descriptor( ibConf_t p, int ud )
{
	int i;
	
	//yuan remove 01/13/12
	///pthread_mutex_lock( m_lock[p.settings.board] );
	if( ud < 0 )
	{
		//yuan add 01/13/12
		pthread_mutex_lock( m_lock[p.settings.board] );
		for( i = GPIB_MAX_NUM_BOARDS; i < GPIB_CONFIGS_LENGTH; i++ )
		{
			if( ibConfigs[ i ] == NULL ) break;
		}
		if( i == GPIB_CONFIGS_LENGTH )
		{
			setIberr( ENEB ); // ETAB?
			pthread_mutex_unlock( m_lock[p.settings.board] );
			return -1;
		}
		ud = i;
		//yuan modify 01/13/12
		ibConfigs[ ud ] = malloc( sizeof( ibConf_t ) );
		memset(ibConfigs[ ud ], 0, sizeof( ibConf_t ));
		if( ibConfigs[ ud ] == NULL )
		{
			setIberr( EDVR );
			setIbcnt( ENOMEM );
			pthread_mutex_unlock( m_lock[p.settings.board] );			
			return -1;
		}
		//...init_ibconf(ibConfigs[ ud ]);
		*ibConfigs[ud] = p;
		//yuan add 01/13/12
		pthread_mutex_unlock( m_lock[p.settings.board] );
		return ud;

	}else
	{
		if( ud >= GPIB_CONFIGS_LENGTH )
		{			
			setIberr( EDVR );
			setIbcnt( EINVAL );
			//yuan remove 01/13/12
			///pthread_mutex_unlock( m_lock[p.settings.board] );
			return -1;
		}
		if( ibConfigs[ ud ] )
		{
			setIberr( EDVR );
			setIbcnt( EINVAL );
			//yuan remove 01/13/12
			///pthread_mutex_unlock( m_lock[p.settings.board] );
			return -1;
		}
	}
	ibConfigs[ ud ] = malloc( sizeof( ibConf_t ) );
	memset(ibConfigs[ ud ], 0, sizeof( ibConf_t ));	
	if( ibConfigs[ ud ] == NULL )
	{
		fprintf( stderr, "libgpib: out of memory\n" );
		setIberr( EDVR );
		setIbcnt( ENOMEM );
		pthread_mutex_unlock( m_lock[p.settings.board] );
		return -1;
	}
	//////init_ibconf(ibConfigs[ ud ]);	
	*ibConfigs[ud] = p;
	pthread_mutex_unlock( m_lock[p.settings.board] );
//printf("ud: %d\n", ud);
	return ud;
}

int setup_board_descriptors( void )
{
	int i, ud=0;
	int retval = 0;

	for( i = 0; i < FIND_CONFIGS_LENGTH; i++ )
	{
		if(ibFindConfigs[ i ].is_interface && ibFindConfigs[ i ].settings.board >= 0 &&
			ibFindConfigs[ i ].settings.board < GPIB_MAX_NUM_BOARDS)
		{
			if((ud = add_descriptor( ibFindConfigs[ i ], ibFindConfigs[ i ].settings.board)) < 0 )
			{
				retval = -1;
			}
			else {
				ibConf_t *conf;				
				ibBoard[i].ud = ud;  
				//yuan add 09/27/06 for testing
				conf = ibConfigs[ ud ];
			}
		}
	}
	
	for( i = 0; i < GPIB_MAX_NUM_BOARDS; i++ )
		if( ibConfigs[ i ] )
			ibConfigs[ i ]->handle = 0;
	return retval;
}

/**********************************************************************/

int ibGetDescriptor( ibConf_t p )
{
	int retval;

	/* XXX should go somewhere else XXX check validity of values */
	if(p.settings.pad > gpib_addr_max || p.settings.sad > gpib_addr_max)
	{
		setIberr( ETAB );
		return -1;
	}
	//yaun add 01/13/12
	if(ibFindConfigs[ p.settings.board ].settings.board >= 0)
	{
	 retval = add_descriptor( p, -1 );
	 if( retval < 0 )
		return retval;
	} else 
	{
		retval = -1;	
		setIberr( ENEB );
	}
	return retval;
}

int ibGetIndexFromName( const char *name )
{
	int i;

	if( strcmp( "", name ) == 0 ) return -1;

	for(i = 0; i < FIND_CONFIGS_LENGTH; i++)
	{
		if(!strcmp(ibFindConfigs[i].name, name)) return i;
	}

	return -1;
}

static int CheckDescriptor( int ud )
{
	if( ud < 0 || ud >= GPIB_CONFIGS_LENGTH || ibConfigs[ud] == NULL )
	{
		fprintf( stderr, "invalid descriptor\n" );
		//setIberr( EDVR );
		setIberr( ENOL );
		setIbcnt( EINVAL );
		return -1;
	}

	return 0;
}

void init_descriptor_settings( descriptor_settings_t *settings )
{
	settings->pad = -1;
	settings->sad = -1;
	settings->board = -1;
	settings->usec_timeout = 3000000;
	settings->spoll_usec_timeout = 1000000;
	settings->ppoll_usec_timeout = 2;
	settings->eos = 0;
	settings->eos_flags = 0;
	settings->ppoll_config = 0;
	settings->send_eoi = 1;
	settings->local_lockout = 0;
	settings->local_ppc = 0;
	settings->readdr = 0;

}

void init_ibconf( ibConf_t *conf )
{
	conf->handle = -1;
	memset(conf->name, 0, sizeof(conf->name));
	init_descriptor_settings( &conf->defaults );
	init_descriptor_settings( &conf->settings );
	memset(conf->init_string, 0, sizeof(conf->init_string));
	conf->flags = 0;
	init_async_op( &conf->async );
	init_asyno_op( &conf->asyno );
	conf->end = 0;
	conf->is_interface = 0;
	conf->board_is_open = 0;
	conf->has_lock = 0;
	conf->timed_out = 0;
	//yuan add 04/17/06
	conf->end_f = 0;
	//conf->tmp_string[0] = 0;
	memset(conf->tmp_string, 0, sizeof(conf->tmp_string));
	conf->flags = 0;
}

int open_device_handle( ibConf_t *conf )
{
	open_dev_ioctl_t open_cmd;
	int retval;
	ibBoard_t *board;

	if( conf->handle >= 0 ) return 0;
	memset( &open_cmd, 0, sizeof(open_dev_ioctl_t) );
	board = interfaceBoard( conf );

	open_cmd.handle = -1;
	open_cmd.pad = conf->settings.pad;
	open_cmd.sad = conf->settings.sad;
	open_cmd.is_board = conf->is_interface;
	open_cmd.usec_timeout = (conf->settings.spoll_usec_timeout>=LINETIMEOUT)? (conf->settings.spoll_usec_timeout+ADDTIMEOUT):conf->settings.spoll_usec_timeout;
	//yuan add 10/20/06
	if(!open_cmd.usec_timeout)
		open_cmd.usec_timeout = 0xffffffff;
	retval = ioctl( board->fileno, IBOPENDEV, &open_cmd );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: IBOPENDEV ioctl failed\n" );
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}
	conf->handle = open_cmd.handle;
        if((!conf->is_interface) && ( check_cic( board ) == 0 )) {
		//printf("open0\n");
		assert_ifc( board, 100 );
		board->assert_ifc = 1;
	}
	return 0;
}

int close_device_handle( ibConf_t *conf )
{
	close_dev_ioctl_t close_cmd;
	int retval;
	ibBoard_t *board;

	if( conf->handle < 0 ) return 0;
        if( conf->is_interface ) return 0;

        memset( &close_cmd, 0, sizeof(close_dev_ioctl_t) );
	board = interfaceBoard( conf );

	close_cmd.handle = conf->handle;
	retval = ioctl( board->fileno, IBCLOSEDEV, &close_cmd );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	conf->handle = -1;

	return 0;
}

int change_gpib_address( ibConf_t *conf, unsigned int pad, int sad, int force )
{
	int retval;
	ibBoard_t *board;
	pad_ioctl_t pad_cmd;
	sad_ioctl_t sad_cmd;

	board = interfaceBoard( conf );

	if((conf->settings.pad != pad) || force) {
	pad_cmd.handle = conf->handle;
	pad_cmd.pad = pad;
	retval = ioctl( board->fileno, IBPAD, &pad_cmd );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}
	conf->settings.pad = pad;
	}
    if((conf->settings.sad != sad) || force) {
	sad_cmd.handle = conf->handle;
	sad_cmd.sad = sad;
	retval = ioctl( board->fileno, IBSAD, &sad_cmd );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}
	conf->settings.sad = sad;
	}
	
	

	return 0;
}

ibConf_t * func_init( int ud )
{
	return _func_init( ud, 0, 0 );
}

ibConf_t * _func_init( int ud, int no_lock_board, int ignore_eoip )
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;

//printf("enter _func_init\n");
	/*retval = ibParseConfigFile();
	if(retval < 0)
	{
		return NULL;
	}*/
	setIberr( 0 );
	///setIbcnt( 0 );
	if( CheckDescriptor( ud ) < 0 )
	{
		return NULL;
	}
	conf = ibConfigs[ ud ];
	retval = conf_online( conf, 1 );
	if( retval < 0 ) return NULL;

	conf->timed_out = 0;

	board = interfaceBoard( conf );

	if( no_lock_board == 0 )
	{
		if( ignore_eoip == 0 )
		{
			pthread_mutex_lock( &conf->async.lock );
			if( conf->async.in_progress && (!(conf->async.ibsta & CMPL)))
			//if( conf->async.in_progress )
			{				
				setIberr( EOIP );
				pthread_mutex_unlock( &conf->async.lock );
				return NULL;
			}
			pthread_mutex_unlock( &conf->async.lock );
		}
	}

	return conf;
}

int ibstatus( ibConf_t *conf, int error, int clear_mask, int set_mask )
{
//printf("enter ibstatus\n");
	int status = 0;
	int retval;
	retval = _ibwait_a( conf, 0, clear_mask, set_mask, &status);
	if( retval < 0 ) error = 1;
	if( error ) status |= ERR;
	if( conf->timed_out )
		status |= TIMO;
	if( conf->end ) {
		status |= END;
		//yuan add 01/17/06
		conf->end = 0;
	}
	////
	//if((!error) && (ThreadIbsta()&CMPL))
	//	status |= CMPL;

	setIbsta( status );
	return status;
}

int func_exit( int ud, int error )
{
	return _func_exit( ud, error, 0, 0, 0, 0, 0 );
}

int _func_exit( int ud, int error, int no_sync_globals, int no_update_ibsta,
	int status_clear_mask, int status_set_mask, int no_unlock_board )
{
	ibConf_t *conf = ibConfigs[ ud ];
	ibBoard_t *board;
	int status;
	if( CheckDescriptor( ud ) < 0 )
	{
		setIbsta( ERR );
		if( no_sync_globals == 0 )
			sync_globals();
		return ERR;
	}

	board = interfaceBoard( conf );

	if( no_update_ibsta )
		status = ThreadIbsta();
	else
		status = ibstatus( conf, error, status_clear_mask, status_set_mask );
	if( no_sync_globals == 0 )
		sync_globals();
	return status;
}

int _getPAD( Addr4882_t address )
{
	int pad = address & 0xff;

	if( address == NOADDR ) return ADDR_INVALID;

	if( pad < 0 || pad > gpib_addr_max ) return ADDR_INVALID;

	return pad;
}

int _getSAD( Addr4882_t address )
{
	int sad = ( address >> 8 ) & 0xff;

	if( address == NOADDR ) return ADDR_INVALID;

	if( sad == NO_SAD ) return SAD_DISABLED;

	if( ( sad & 0x60 ) == 0 ) return ADDR_INVALID;

	sad &= ~0x60;

	if( sad < 0 || sad > gpib_addr_max ) return ADDR_INVALID;

	return sad;
}

Addr4882_t _mkAddr( unsigned int pad, int sad )
{
	Addr4882_t address;

	address = 0;
	address |= pad & 0xff;
	if( sad >= 0 )
		address |= ( ( sad | sad_offset ) << 8 ) & 0xff00;

	return address;
}

int check_address( Addr4882_t address )
{
	if( address == NOADDR ) return 1;

	if( _getPAD( address ) == ADDR_INVALID ||
		_getSAD( address ) == ADDR_INVALID )
	{
		setIberr( EARG );
		return 0;
	}

	return 1;
}

int check_addressList( const Addr4882_t addressList[] )
{
	int i;

	if( addressList == NULL ) return 1;

	for( i = 0; addressList[ i ] != NOADDR; i++ )
	{
		if( check_address( addressList[ i ] ) == 0 )
		{
			setIbcnt( i );
			return 0;
		}
	}

	return 1;
}

unsigned int numofAddresses( const Addr4882_t addressList[] )
{
	unsigned int count;

	if( addressList == NULL )
		return 0;

	count = 0;
	while( addressList[ count ] != NOADDR )
	{
		count++;
	}

	return count;
}

int check_cic( const ibBoard_t *board )
{
	int retval;
	wait_ioctl_t cmd;
#define HR_CIC ( 1 << 7 )
	cmd.usec_timeout = 0;
	cmd.wait_mask = 0;
	cmd.clear_mask = 0;
	cmd.set_mask = 0;
	cmd.pad = NOADDR;
	cmd.sad = NOADDR;
	cmd.handle = 0;
	cmd.ibsta = 0;
	retval = ioctl( board->fileno, IBSTA, &cmd );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		fprintf( stderr, "libgpib: error in check_cic()!\n");
		return -1;
	}
	if( cmd.ibsta & HR_CIC)
		return 1;
	return 0;
}

int check_sc( const ibBoard_t *board )
{
	int retval;
	board_info_ioctl_t info;

	retval = ioctl( board->fileno, IBBOARD_INFO, &info );
	if( retval < 0 )
	{
		fprintf( stderr, "libgpib: error in check_sc()!\n");
		return retval;
	}

	return info.is_system_controller;
}

int parse_board_t1_delay( int t1_delay )
{
        if( t1_delay < 500 ) return T1_DELAY_350ns;
	else if( t1_delay < 2000 ) return T1_DELAY_500ns;
	return T1_DELAY_2000ns;
}
