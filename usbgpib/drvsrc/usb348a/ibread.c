/***************************************************************************
 ibread.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from cb7210/cb7210_read.c and sys/ibread.c of 
         the Linux GPIB Package driver by Frank Mori Hess      
 Copyright (C) 2001, 2002, 2003, 2004 by 
         Frank Mori Hess <fmhess@users.sourceforge.net>
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
#include <linux/sched.h>
#include <linux/usb.h>
#include "../include/Adllib.h"
#include "../include/gpib_ioctl.h"
#include "gpib_proto.h"
#include "gpib_user.h"
#include "../include/Adlp9030.h"
#include "u348a.h"
#include "ezusbsys.h"

/*----------------------------------------------------------------------------------*/

/*
 * IBRD
 * Read up to 'length' bytes of data from the GPIB into buf.  End
 * on detection of END (EOI and or EOS) and set 'end_flag'.
 *
 * NOTE:
 *      1.  The interface is placed in the controller standby
 *          state prior to beginning the read.
 *      2.  Prior to calling ibrd, the intended devices as well
 *          as the interface board itself must be addressed by
 *          calling ibcmd.
 */

extern long cb7210_fifo_read( PU348A_DEVEXT board, gpib_descriptor_t * desc, uint8_t *buffer,
	size_t length, int *end );

int ibrd( PU348A_DEVEXT board, gpib_descriptor_t * desc, uint8_t *buf, size_t length, int *end_flag )
{
	size_t count = 0, tmpcnt=0;
	int ret = 0;
	BULK_TRANSFER_CONTROL bulkControl;
  	uint8_t retval[16];
	uint8_t buffer[8];	
	unsigned long usec_timeout=0;
	size_t remaincnt = 0, bulkcnt=0, totalcnt=length;
	int bulkend, rderr=0;
	uint8_t hsmode_org = 0;
	uint8_t is_cic = 0;
	int res = 0;
	
	hsmode_org = board->auxa_bits & HR_HANDSHAKE_MASK;
	
	if( length == 0 )
	{
		printk( "gpib: ibrd() called with zero length?\n");
		return 0;
	}
	/////is_cic = (read_byte(board, ADSR)& HR_CIC)? 1:0;
	/* XXX reseting timer here could cause timeouts take longer than they should,
	 * since read_ioctl calls this
	 * function in a loop, there is probably a similar problem with writes/commands */
    //if(board->hs_mode_bits & HS_RX_ENABLE)
	   //....osStartTimer( board, board->usec_timeout );
	//do
	{
		//if(board->hs_mode_bits & HS_RX_ENABLE) 
		{
			board->hs_mode_bits |= HS_RX_DIR;
			//.....ret = cb7210_fifo_read(board, desc, buf, length - count, end_flag);
		      //yuan add 03/09/10
		  usec_timeout = board->usec_timeout/1000;
		  if(board->usec_timeout)
		  {						
			  usec_timeout = (usec_timeout<1)? 1: usec_timeout; 
		  }
			buffer[0] = (uint8_t) (usec_timeout & 0xff);
			buffer[1] = (uint8_t) ((usec_timeout>>8) & 0xff);
			buffer[2] = (uint8_t) ((usec_timeout>>16) & 0xff);
			buffer[3] = (uint8_t) ((usec_timeout>>24) & 0xff);
			buffer[4] = (uint8_t) (length & 0xff);
			buffer[5] = (uint8_t) ((length>>8) & 0xff);
			buffer[6] = (uint8_t) ((length>>16) & 0xff);
			buffer[7] = (uint8_t) ((length>>24) & 0xff);
    	res = UDServiceFire(board, VR_GPIB_DI, (1<<8)|is_cic/*board->master*/, board->auxa_bits /*0x92*/, 8, buffer, 0);	
			if (res<0) {
				return -1;
			}
			*end_flag = 0;
			bulkControl.ep = 0x86;
			bulkControl.pipeNum = usb_rcvbulkpipe(board->udev,bulkControl.ep); 
			//bulkControl.pipeNum = 4; 
			//...if(length>=4000)
				//...length = 4000;
			tmpcnt = 0;
			bulkend = 0;
		//yuan modify 03/09/10
		if(board->usec_timeout)
			wdt_start( board, board->usec_timeout+1000000 );	
		do {
			if ((tmpcnt < length) && (!bulkend)) 
			{
				bulkcnt = 0;
				res = Ezusb_ReadWrite(board, &bulkControl, (buf+tmpcnt), totalcnt, URB_DIR_IN, &bulkcnt, &bulkend);
				if(res<0) {
					//yuan modify 03/09/10
					if(board->usec_timeout)
						wdt_remove(board);
					UDServiceFire(board, VR_GPIB_ABORT, 0, 0, 0, NULL, 0);
					return -1;
				}					
				tmpcnt += bulkcnt;
				totalcnt -= bulkcnt;
				//printk("bulk rd: %d\n", bulkcnt);
			}
			memset(retval, 0, sizeof(retval));		
			GetGPIBResponse(board, 11, retval);
		} while((retval[0] != DONE) && (!test_bit( TIMO_NUM, &board->status )));
		//yuan modify 03/09/10
		if(board->usec_timeout)
			wdt_remove(board);
		if(test_bit( TIMO_NUM, &board->status ) && (retval[0] == BUSY)) {
			ret = 0;
			UDServiceFire(board, VR_GPIB_ABORT, 0, 0, 0, NULL, 0);				
		} 	
		if(retval[0] == DONE) 
		{ 
	//KdPrint(("fifo1: %x\n", read_byte(board, HS_STATUS)));
			ret = (retval[5]<<24)|(retval[4]<<16)|(retval[3]<<8)|(retval[2]);
			*end_flag = retval[6];
			if((*end_flag)==1)
				set_bit(RECEIVED_END_BN, &board->state);
			//...board->dwDevRA = ret;
			board->dwDevRA = (tmpcnt>length)? length:tmpcnt;
			//check data in FIFO
	   		remaincnt = 0;/////(retval[10]<<24)|(retval[9]<<16)|(retval[8]<<8)|(retval[7]);
	   		//if((ret<0) && (((*end_flag)==1)||(board->dwDevRA==length)))
	   			//ret = board->dwDevRA;
	   		
	   	} else {
	   		rderr = 1;
	   		ret = 0;
	   		board->dwDevRA = (tmpcnt>length)? length:tmpcnt;//ret;	   		
	   		//if((ret<0) && (((*end_flag)==1)||(ret==length)))
	   		//	ret = board->dwDevRA;
	   	}	

			//...board->dwDevRA = ret;
			if((retval[0] == DONE) && ((ret==length) || (*end_flag == 1))) {
				if(*end_flag)
					set_bit(RECEIVED_END_BN, &board->state);
				////else
					/////read_byte( board, DIR );
	   		//*retcnt = buflen;	   					
			} else { 
	   		//return retval[1];	   		
	   		ret = ret? ret:-1;
	   		if(retval[0] == DONE) {	 
	   		    if(retval[1]<0)
	   			ret = retval[1];		
	   		    else if(!retval[1])
	   		    	ret = -100;
	   		    else
	   		    	ret = -1*retval[1];
	   		} else if(ret <0)
	   			ret = -1;
	   		if((retval[1] == rtn_Timeout) ||(retval[1] == rtn_Abort) )
	   			ret = -ETIMEDOUT;
		    	else if(retval[1] == rtn_Err)
			   	return -3;
			//else 
			//	return -1;
			//if(retval[7] !=0)
			//	return -1*retval[7];
			//else return -1;
	   		//else ret = -1;
			}		
		} /*else {
			board->dwDevRB = 0;
			osStartTimer( board, board->usec_timeout );
			ret = nec7210_read(board, desc, buf, length - count, end_flag);
			osRemoveTimer(board);
		}*/
		if(ret < 0)
		{
			printk("gpib read error %d\n", ret);
		}else
		{
			buf += ret;
			count += ret;
		}
	}//while(ret > 0 && count < length && *end_flag == 0);
	return (ret < 0) ? ret : count;
}

