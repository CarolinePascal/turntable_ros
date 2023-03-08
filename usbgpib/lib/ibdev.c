/***************************************************************************
 ibdev.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibdev.c of the Linux GPIB Package driver 
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

int ibdev( int minor, int pad, int sad, int timo, int eot, int eosmode )
{
	int retval;
	ibConf_t new_conf;
	int err=0, sta=0;

	/*retval = ibParseConfigFile();
	if(retval < 0)
	{
		setIbsta( ERR );
		return -1;
	}*/

	//yuan add 01/13/12
	pad &= 0xff;
	if((pad<0) || (pad>30)) {
		//setIbsta( CMPL | ERR );
		sta = ERR;
		err = EARG;
		setIbsta( ERR );
		setIberr( EARG );
		pad = 0x1e;
		/*sync_globals();
		if(spyR) {
		  if(rec) {
			GetLocalTime(&ttime);
     		RecordPost(recID, &tmpR, &stime, &ttime);
		  }
		}
		return -1;*/
	}
	if(sad) {
		//yuan add 05/18/11
	 sad &= 0xff;
	 sad -= sad_offset;
	 //yuan add 08/11/08
	 if((sad<0) || (sad>30)) {
		setIbsta( ERR );
		setIberr( EARG );
		sta = ERR;
		err = EARG;
		sad = SAD_DISABLED;
		/*sync_globals();
		if(spyR) {
		  if(rec) {
			GetLocalTime(&ttime);
     		RecordPost(recID, &tmpR, &stime, &ttime);
		  }
		}
	   return -1;*/
	 }
	} else //yuan add 07/02/09
		sad = SAD_DISABLED;
	//sad -= sad_offset;
	init_ibconf( &new_conf );
	new_conf.settings.pad = pad;
	new_conf.settings.sad = sad;                        /* device address                   */
	new_conf.settings.board = minor;                    /* board number                     */
	new_conf.settings.eos = eosmode & 0xff;             /* local eos modes                  */
	new_conf.settings.eos_flags = eosmode & 0xff00;
	//yuan modify for testing
	new_conf.settings.usec_timeout = timeout_to_usec( timo );
	//new_conf.settings.usec_timeout = ibConfigs[ibBoard[minor].ud]->settings.usec_timeout;
	if( eot )
		new_conf.settings.send_eoi = 1;
	else
		new_conf.settings.send_eoi = 0;
	new_conf.defaults = new_conf.settings;
	new_conf.is_interface = 0;
	
	//yuan modify 01/13/12
	retval = _ibdev( new_conf );
	setIbsta( sta|ibsta );
	setIberr( err|iberr );
	sync_globals();

	return retval;//_ibdev( new_conf );
}

int _ibdev( ibConf_t new_conf )
{
	int ud;
	ibConf_t *conf;
//printf("new_conf.is_interface: %d %d\n",new_conf.is_interface, new_conf.settings.board); 
	if(new_conf.is_interface)
	   ud = new_conf.settings.board;
	else 
	   ud = ibGetDescriptor(new_conf);
//printf("_ibdev: %d\n", ud);
	if( ud < 0 )
	{
		//fprintf( stderr, "ibdev failed to get descriptor\n" );
		//yuan modify 01/13/12
		setIbsta( ERR );
		setIberr( EDVR );
		sync_globals();
		return -1;
	}
	conf = func_init( ud );
	if( conf == NULL )
	{
		//yuan modify 01/13/12
		setIberr( EDVR );
		func_exit( ud, 1 );
		return -1;
	}
	func_exit( ud, 0 );
	return ud;
}
