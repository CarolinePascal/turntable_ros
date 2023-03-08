/***************************************************************************
 ibwrite.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from cb7210/cb7210_write.c and sys/ibwrite.c of the 
 	Linux GPIB Package driver by Frank Mori Hess      
 Copyright (C) 2001, 2002, 2003 by Frank Mori Hess <fmhess@users.sourceforge.net>  
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include "../include/Adllib.h"
#include "../include/gpib_ioctl.h"
#include "gpib_proto.h"
#include "gpib_user.h"
#include "../include/Adlp9030.h"
#include "u348a.h"
#include "ezusbsys.h"

/*
 * IBWRT
 * Write cnt bytes of data from buf to the GPIB.  The write
 * operation terminates only on I/O complete.
 *
 * NOTE:
 *      1.  Prior to beginning the write, the interface is
 *          placed in the controller standby state.
 *      2.  Prior to calling ibwrt, the intended devices as
 *          well as the interface board itself must be
 *          addressed by calling ibcmd.
 */
int ibwrt(PU348A_DEVEXT board, gpib_descriptor_t * desc, uint8_t *buf, size_t cnt, int send_eoi)
{
	int bytes_sent = 0;
        size_t bulkcnt=0;
	int ret = 0;
	//int retval;
	BULK_TRANSFER_CONTROL bulkControl;
  	uint8_t retval[6];
	uint8_t buffer[8];	
	unsigned long usec_timeout=0;
	int bulkend;
	uint8_t is_cic = 0;
	int res = 0;

	if( cnt == 0 )
	{
		//printk( "gpib: ibwrt() called with zero length?\n" );
		return 0;
	}
	{
		clear_bit( DEV_CLEAR_BN, &board->state );
		clear_bit(WRITE_READY_BN, &board->state);
		board->hs_mode_bits &= ~HS_RX_DIR;
		//ret = cb7210_fifo_write(board, desc, buf, cnt, send_eoi);
		//yuan modify 03/09/10
		usec_timeout = board->usec_timeout/1000;		
		if(board->usec_timeout)
		{
			usec_timeout = (usec_timeout<1)? 1: (usec_timeout); // timeout*100 unit: (10us);			
		}
		buffer[0] = (unsigned char) (usec_timeout & 0xff);
		buffer[1] = (unsigned char) ((usec_timeout>>8) & 0xff);
		buffer[2] = (unsigned char) ((usec_timeout>>16) & 0xff);
		buffer[3] = (unsigned char) ((usec_timeout>>24) & 0xff);
		buffer[4] = (unsigned char) (cnt & 0xff);
		buffer[5] = (unsigned char) ((cnt>>8) & 0xff);
		buffer[6] = (unsigned char) ((cnt>>16) & 0xff);
		buffer[7] = (unsigned char) ((cnt>>24) & 0xff);
    	res = UDServiceFire(board, VR_GPIB_DO, (send_eoi<<8)|is_cic/*board->master*/, 0, 8, buffer, 0);	
		if (res<0) {
			return -1;
		}
		//bulkControl.pipeNum = 2; //?????
		bulkControl.ep = 2;
		bulkControl.pipeNum = usb_sndbulkpipe(board->udev,bulkControl.ep); 
		res = Ezusb_ReadWrite(board, &bulkControl, buf, cnt, URB_DIR_OUT, &bulkcnt, &bulkend);
		//printk("wrtrw: %d %d %x\n", cnt, bulkcnt, bulkend);
	  if (res<0) {
		UDServiceFire(board, VR_GPIB_ABORT, 0, 0, 0, NULL, 0);
		//reset pipe
		res = usb_clear_halt(board->udev, bulkControl.pipeNum);
#if 0
		Ezusb_ResetPipe(
   			board,
   			bulkControl.pipeNum
  		);
#endif  		
		return -1;
	}		
				//return cnt;
		memset(retval, 0, sizeof(retval));
		//yuan modify 03/09/10
		if(board->usec_timeout)
			wdt_start( board, board->usec_timeout+4000000 );
		do {
			GetGPIBResponse(board, 6, retval);
		} while((retval[0] == BUSY) && (!test_bit( TIMO_NUM, &board->status )));
		//yuan modify 03/09/10
		if(board->usec_timeout)		
			wdt_remove( board );
		//ret = (retval[3]<<8)|(retval[2]);
		if(retval[0] == BUSY) {
			ret = 0;
			UDServiceFire(board, VR_GPIB_ABORT, 0, 0, 0, NULL, 0);	
		} else 
			ret = (retval[5]<<24)|(retval[4]<<16)|(retval[3]<<8)|(retval[2]);
		//printk("wrt resp: %d %d %d\n", retval[0], retval[1], ret);
		if((retval[0] == DONE) && (ret==cnt)) {
	   		//*retcnt = buflen;	   	
		} else 
		{ 
	   		//return retval[1];
	   		ret = ret? ret:-1;	   		
	   		if(retval[1] == rtn_Timeout || io_timed_out( board ) )
	   			ret = -4;
			if(retval[1] == rtn_Err)
	   			ret = -3;
			else ret = -1;

			res = usb_clear_halt(board->udev, bulkControl.pipeNum);
		}		
	} 
	if(ret < 0)
	{
		printk("write error\n");
		//printk("gpib write error\n");
	}else
	{
		buf += ret;
		bytes_sent += ret;
	}
	//osRemoveTimer(board);
	if(ret < 0) return ret;

	return bytes_sent;
}
