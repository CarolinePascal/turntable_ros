/***************************************************************************
 device.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/device.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2001, 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include <linux/delay.h> 
#include <linux/usb.h>
#include "gpib_proto.h"
#include "gpib_user.h"
#include "../include/Adldev.h"
#include "../include/Adllib.h"
#include "autopoll.h"
#include "u348a.h"
#include "ezusbsys.h"


int serial_polls( PU348A_DEVEXT board, unsigned int spcnt, char* buf, uint8_t *result, unsigned int* retcnt )
{
  int i=0;
  BULK_TRANSFER_CONTROL bulkControl;
  unsigned int buflen = 6*spcnt;
  size_t bulkcnt=0;
  int bulkend;
	unsigned char retval[6];
	int ret = 0;
  uint8_t *spbuf = NULL;
	
	//printk("serial polls2: %d %d\n", buf[4], buf[5]);
	*retcnt = 0;
	spbuf = kmalloc( spcnt, GFP_KERNEL );
	if( spbuf == NULL )
		return -ENOMEM;
	memset(spbuf, 0, spcnt);
  ret = UDServiceFire(board, VR_GPIB_CTRL_SP, (board->sad<<16)|board->pad, 0x91, buflen, buf, 0);	
  if (ret < 0) 
  {  
      dev_err(&board->udev->dev, "U348A - %s - retval=%d\n",  __FUNCTION__, ret ); 
      return -1;
  }	  

	bulkControl.ep = 0x88;
	bulkControl.pipeNum = usb_rcvbulkpipe(board->udev,bulkControl.ep); 
//printk("sp: %x\n", bulkControl.pipeNum);
	//bulkControl.pipeNum = 5; //?????
	Ezusb_ReadWrite(board, &bulkControl, /*result*/spbuf, spcnt, URB_DIR_IN, &bulkcnt, &bulkend);
	//yuan modify 03/09/10
	if(board->usec_timeout)	
		wdt_start( board, board->usec_timeout+4000000 );
		//osStartTimer( board, board->usec_timeout+4000000 );
	do {
		GetGPIBResponse(board, 6, retval);
	} while((retval[0] == BUSY) && (!test_bit( TIMO_NUM, &board->status )));
	//yuan modify 03/09/10
	if(board->usec_timeout)	
		wdt_remove( board );//osRemoveTimer( board );
	buflen = (retval[3]<<8)|(retval[2]);
	*retcnt = buflen;
	if((retval[0] == DONE) && (buflen ==spcnt)) {
	   //*retcnt = buflen;
	   //printk("VR_GPIB_CTRL_SP end: %x\n", buflen);	   	
	} else { 
   	   if(spbuf != NULL)
		kfree( spbuf );
	   //return retval[1];
	   if(retval[1] == rtn_Timeout)
	   {
	   	printk("serial polls timeout: %d %d\n", retval[0], retval[1]);
	   	return -4;
	   }
	   else return -1;
	}
	if( spbuf != NULL ) {
		memcpy(result, spbuf, spcnt);
		kfree( spbuf );
	}
		
	return 0;
}

static int serial_poll_single( PU348A_DEVEXT board, unsigned int pad, int sad,
	unsigned int usec_timeout, uint8_t *result )
{
	int retval;
  	unsigned int retcnt=0;
	uint8_t buffer[6];	
	uint8_t hsmode_org = 0;
	
	hsmode_org = board->auxa_bits & HR_HANDSHAKE_MASK;
	
	retval = 0;
	if(usec_timeout)
	{
		usec_timeout = usec_timeout/1000;		
		usec_timeout = (usec_timeout<1)?1: usec_timeout;
	}
	buffer[0] = usec_timeout & 0xff;
	buffer[1] = (usec_timeout>>8) & 0xff;
	buffer[2] = (usec_timeout>>16) & 0xff;
	buffer[3] = (usec_timeout>>24) & 0xff;
	buffer[4] = pad & 0xff;
	buffer[5] = (sad>=0)? (sad&0xff): -1;
	//printk("serial polls: %d %d\n", buffer[4], buffer[5]);
	retval = serial_polls(board, 1, buffer, result, &retcnt);  
	board->dwDevRA = 0;
	board->dwDevRB = 0;
	if(*result & 0x40) {
		board->status &= ~SRQI;
	}

	board->cur_taddr = UNT;

	return retval;
}

int serial_poll_all( PU348A_DEVEXT board, unsigned int usec_timeout )
{
	int retval;
	struct list_head *cur;
	const struct list_head *head = &board->device_list;
	gpib_status_queue_t *device;
	unsigned int num_bytes=0, i=0, j=0;
	uint8_t buffer[128*6];
	uint8_t result[128];
	unsigned int retcnt=0;	
	U8 hsmode_org = 0;
	
	//printk( "entering serial_poll_all()\n" );
	memset(buffer, 0, sizeof(buffer));
	memset(result, 0, sizeof(result));
	//if( head->Flink == head ) return 0;
	if( head->next == head ) return 0;
	//retval = setup_serial_poll( board, usec_timeout );
	//if( retval < 0 ) return retval;
	hsmode_org = board->auxa_bits & HR_HANDSHAKE_MASK;
	num_bytes = 0;
	retval = 0;
	j=0;

	for( cur = head->next; cur != head; cur = cur->next )
	{
#if 0
		retval = serial_poll_enable( board, usec_timeout );
		if( retval < 0 ) continue;//..return retval;
#endif
	//for( cur = head->Flink; cur != head; cur = cur->Flink )
	//{
		device = list_entry( cur, gpib_status_queue_t, list );
		usec_timeout = device->usec_timeout;
		if(usec_timeout)
		{		
			usec_timeout = device->usec_timeout/1000;		
			usec_timeout = (usec_timeout<1)?1: usec_timeout;
		}
		//usec_timeout = (device->usec_timeout<10)? 1: (device->usec_timeout/10); // timeout*100 unit: (10ns);			
		buffer[i++] = usec_timeout & 0xff;
		buffer[i++] = (usec_timeout>>8) & 0xff;
		buffer[i++] = (usec_timeout>>16) & 0xff;
		buffer[i++] = (usec_timeout>>24) & 0xff;
		buffer[i++] = device->pad & 0xff;
		buffer[i++] = (device->sad>=0)? (device->sad&0xff): -1;
		j++;
	}
	
	retval = serial_polls(board, j, buffer, result, &retcnt);  				
		
	i=0;
	for( cur = head->next; cur != head; cur = cur->next )
	{
		if(i>=retcnt)
			break;		
		device = list_entry( cur, gpib_status_queue_t, list );
		if( result[i] & request_service_bit )
		{
			board->status &= ~SRQI;
			retval = push_status_byte( device, result[i] );			
			//if(i==retcnt)
			//	break;
			if( retval < 0 ) continue;
			num_bytes++;
		}
		i++;
	}
	board->cur_taddr = UNT;
	return num_bytes;
}


int dvrsp( PU348A_DEVEXT board, unsigned int pad, int sad,
	unsigned int usec_timeout, uint8_t *result )
{	
	int retval;
  
  if( ((int) pad) > gpib_addr_max || ((int) sad) > gpib_addr_max )
	{
		printk("gpib: bad address for serial poll");
		return -1;
	}

	retval = serial_poll_single( board, pad, sad, usec_timeout, result );
	
	return retval;
}

