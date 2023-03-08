/***************************************************************************
 ibcmd.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/ibcmd.c of the Linux GPIB Package driver 
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
#include "../include/Adllib.h"
#include "../include/gpib_ioctl.h"
#include "gpib_proto.h"
#include "gpib_user.h"
#include "u348a.h"
#include "ezusbsys.h"

 // IBCMD
 
int ibcmd( PU348A_DEVEXT board, gpib_descriptor_t * desc, uint8_t *buf, size_t length )
{
	size_t	count = 0, bulkcnt=0;
	//int status = ibstatus( board );
  	BULK_TRANSFER_CONTROL bulkControl;
    //unsigned long buflen;
	unsigned char retval[10];
	unsigned char buffer[8];	
	unsigned long usec_timeout=0;
	int bulkend;
	size_t tmplen=0;
	int res = 0;
	
	if(length == 0) return 0;
	tmplen=length;
	length = (tmplen>512)? 512 : tmplen;
	//if((status & CIC) == 0)
	//{
		//printk("gpib: cannot send command when not controller-in-charge\n");
	//	return -1;
	//}
	//yuan modify 03/09/10
	usec_timeout = board->usec_timeout/1000;
	if(board->usec_timeout)
	{		
		usec_timeout = (usec_timeout<1)? 1: usec_timeout; // timeout*100 unit: (10ns);			
        }
	buffer[0] = (uint8_t) (usec_timeout & 0xff);
	buffer[1] = (uint8_t) ((usec_timeout>>8) & 0xff);
	buffer[2] = (uint8_t) ((usec_timeout>>16) & 0xff);
	buffer[3] = (uint8_t) ((usec_timeout>>24) & 0xff);
	buffer[4] = (uint8_t) (length & 0xff);
	buffer[5] = (uint8_t) ((length>>8) & 0xff);
	buffer[6] = (uint8_t) ((length>>16) & 0xff);
	buffer[7] = (uint8_t) ((length>>24) & 0xff);

	res = UDServiceFire(board, VR_GPIB_CTRL_M, board->master?1:0, 0, 8, buffer, 0);	
    //ntstatus = UDServiceFire(board, VR_GPIB_CTRL_M, 0, 0, 8, buffer, 0);	
	if (res<0) {
		return -1;
	}
	//AdlSleep(100);
#if 0
{
int i=0;
for(i=0;i<length;i++)
{
printk("cmd:%x ", buf[i] );
if(i>2) {
printk("\n");
break;}
}
}
#endif
	memset(&bulkControl, 0, sizeof(BULK_TRANSFER_CONTROL));
	bulkControl.ep = 4;
	bulkControl.pipeNum = usb_sndbulkpipe(board->udev,bulkControl.ep); 
	////do {
	////tmplen2 = (tmplen>512)? 512: tmplen;	
	res = Ezusb_ReadWrite(board, &bulkControl, buf, length, URB_DIR_OUT, &bulkcnt, &bulkend);
	if (res<0) {
		UDServiceFire(board, VR_GPIB_ABORT, 0, 0, 0, NULL, 0);
		//for reset pipe
		res = usb_clear_halt(board->udev, bulkControl.pipeNum);
#if 0			
		Ezusb_ResetPipe(
   			board,
   			bulkControl.pipeNum
  		);
#endif  		
		return -1;
	}		
	//printk("cmdrw: %d %d", bulkcnt, bulkend);
	////tmplen = tmplen-tmplen2;
       ////} while(tmplen);
	//AdlSleep(1000);
	memset(retval, 0, sizeof(retval));
	//yuan modify 03/09/10
	if(board->usec_timeout)
		wdt_start( board, board->usec_timeout+1000000 );
	do {
		//yuan modify 03/17/10
		//GetGPIBResponse(board, 6, retval);
		GetGPIBResponse(board, 7, retval);
		//printk("cmd0 response: %d", retval[0]);
	} while((retval[0] == BUSY) && (!test_bit( TIMO_NUM, &board->status )));
	//printk("cmd1 response: %d", retval[0]);
	//yuan modify 03/09/10
	if(board->usec_timeout)	
		wdt_remove( board );
	if(retval[0] == BUSY) {
		printk("abortttttt.....................................");
		UDServiceFire(board, VR_GPIB_ABORT, 0, 0, 0, NULL, 0);	
		memset(retval, 0, sizeof(retval));
		//yuan modify 03/09/10
		if(board->usec_timeout)		
			wdt_start( board, board->usec_timeout );
		do {
			GetGPIBResponse(board, 7, retval);
		} while((retval[0] == BUSY) && (!test_bit( TIMO_NUM, &board->status )));
		//yuan modify 03/09/10
		if(board->usec_timeout)
			wdt_remove( board );
	}
	//yuan add 03/17/10
	if(!(retval[6] & HR_CIC))
		////board->master = 0;
		clear_bit(CIC_NUM, &board->status);
	
	//set_bit(COMMAND_READY_BN, &board->state);
	count = (retval[3]<<8)|(retval[2]);
	//count = (retval[2]<<8)|(retval[3]);
	if((retval[0] == DONE) && (count==length)) {
	   //*retcnt = buflen;	   	
	} else {
		res = usb_clear_halt(board->udev, bulkControl.pipeNum);
#if 0		
		Ezusb_ResetPipe(
   			board,
   			bulkControl.pipeNum
  		);		 
#endif  		
	   //return retval[1];
	   if((retval[1] == rtn_Timeout) || io_timed_out( board ) )
	   	return -4;
	   if(retval[1] == rtn_nCIC)
	   	return -8;		   	
	   if(retval[1] == rtn_Err)
	   	return -3;   	
	   else return -1;
	}
		
	return count;	
}	


