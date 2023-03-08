/***************************************************************************
 ioctl.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from sys/osfuncs.c of the Linux GPIB Package driver
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
 
#include <linux/fcntl.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include <linux/sched.h>
#include <linux/usb.h>
//#include <linux/fs_struct.h>

#include "../include/Adldev.h"
#include "../include/Adllib.h"
#include "gpib_user.h"
#include "../include/gpib_ioctl.h"
#include "gpib_proto.h"
#include "autopoll.h"
#include "u348a.h"
#include "ezusbsys.h"

/*----------------------------------------------------------------------------------*/
extern unsigned int _update_status( PU348A_DEVEXT priv );
extern int fifo_not_empty( PU348A_DEVEXT priv );

/*****************************************************************************/  
/* Forward declaration for our usb_driver definition later.                  */  
/*****************************************************************************/  
int UDServiceFire(
    U348A_DEVEXT *pDevExt,
    uint8_t bRequest,
    uint16_t wValue,
    uint16_t wIndex,
    int nLen,
    void *buf,
    uint8_t In_direction )
{
    int  retval = 0;
    uint8_t  *rwbuf = NULL;
    struct usb_device *udev;
    
    
    if( nLen > 0 )
    {
        rwbuf = kmalloc( nLen, GFP_KERNEL );
        if( rwbuf == NULL )
            return -ENOMEM;
    }
    
    udev = pDevExt->udev;
    down_interruptible(&pDevExt->req_mutex);    
    if( In_direction == DIRECTION_IN )
    {
    	retval = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),  
                   bRequest,   
                   USB_DIR_IN | USB_TYPE_VENDOR,  
                   wValue,   
                   wIndex, 
                   rwbuf, 
                   nLen,  
                   USB_CTRL_GET_TIMEOUT);
                   
    	if (retval < 0) 
    	{  
        goto UDServiceFire_end;
    	}
    	
    	if( rwbuf != NULL )
        memcpy( buf, rwbuf, nLen );
    }
    else
    {
     	if( rwbuf != NULL )   	
        memcpy( rwbuf, buf, nLen );
    	    	
    	retval = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),  
                   bRequest,   
                   USB_DIR_OUT | USB_TYPE_VENDOR,  
                   wValue,   
                   wIndex, 
                   rwbuf, 
                   nLen,  
                   USB_CTRL_SET_TIMEOUT);

    	if (retval < 0) 
    	{  
        goto UDServiceFire_end;
    	}
    }	
    
UDServiceFire_end:
    if( rwbuf != NULL )   
        kfree( rwbuf ); 
    up(&pDevExt->req_mutex);
           	                   	    
    if( retval >= 0 )
        return 0;
    else {
printk("UDServiceFire_%d %d\n", bRequest, retval); 
        return retval;
	}
}   

int GetGPIBResponse(U348A_DEVEXT *priv, uint16_t len, uint8_t *buf)
{
	return UDServiceFire(priv, VR_GPIB_RESPONSE, 0, 0, len, buf, 1);	
}

#if 0
int GetGPIBResponse_bulk(PU348A_DEVEXT priv, uint16_t len, uint8_t *buf)		
{
	int retval = 0;
	BULK_TRANSFER_CONTROL bulkControl;
	unsigned int bulkcnt=0;
    	int bulkend;
	bulkControl.pipeNum = 1; //?????
	retval = Ezusb_ReadWrite(priv, &bulkControl, buf, len, &bulkcnt, &bulkend);
	
	return retval;
}
#endif
int Ezusb_ReadWrite(
   U348A_DEVEXT *dev,
   BULK_TRANSFER_CONTROL *bulkControl,
   uint8_t * bulkbuffer,   
   size_t bufferLength,
   int dir,   
   size_t *actualLength,
   int *end
   )
/*++
Routine Description:
    
Arguments:

Return Value:
    NT status code
        STATUS_SUCCESS:                 Read was done successfully
        STATUS_INVALID_PARAMETER_3:     The Endpoint Index does not specify an IN pipe 
        STATUS_NO_MEMORY:               Insufficient data memory was supplied to perform the READ

    This routine fills the status code into the Irp
    
--*/
{
   size_t  totalbuflen=bufferLength;
   int trlen=0;
   uint8_t *tmpbuffer;
   int retval=0;
struct usb_host_endpoint *ep;
   //printk("enter Ezusb_Read_Write()\n");
   
   if (bufferLength > 0x200)//pipeInfo->MaximumTransferSize)
   {
      bufferLength = 0x200;//pipeInfo->MaximumTransferSize;
   }
   tmpbuffer = bulkbuffer;
   *actualLength = 0;

	ep = usb_pipe_endpoint(dev->udev, bulkControl->pipeNum);


   do {
	/* do a blocking bulk read to get data from the device */
	retval = usb_bulk_msg(dev->udev,
		bulkControl->pipeNum,
		//(dir == URB_DIR_IN)? usb_rcvbulkpipe(dev->udev, bulkControl->ep) : usb_sndbulkpipe(dev->udev,bulkControl->ep),
	      	tmpbuffer,
	      	bufferLength,
	      	&trlen, (dev->usec_timeout)/1000);////HZ*10);
     //
     // If the transfer was successful, report the length of the transfer to the
     // caller by setting IoStatus.Information
     //
    if (!retval) 
    {
   		*actualLength += trlen;
   		if (dir == URB_DIR_IN) {//In only once to check if IN is done
   		  if(trlen < bufferLength)//totalbuflen)//bufferLength) 
   			*end = 1;
	  	  break;
   		}
   		totalbuflen -= bufferLength;
   		tmpbuffer += bufferLength;
   		bufferLength = (totalbuflen>=0x200)? 0x200:totalbuflen;
   	} else {
      	*actualLength = 0;
      	break;
   	}
   } while (totalbuflen);
//printk("Ezusb_ReadWrite_%d: %d\n", retval, *actualLength);
   return retval;
}

void PollingRoutine(U348A_DEVEXT *pdx)
	{							// PollingThreadRoutine
	int retval = 0;

	//printk("POLLING - Starting polling routine %d\n", pdx->autospollers);
	
		//yuan add 10/07/05
		if(pdx->stuck_srq) {
//			ExReleaseFastMutex (&pdx->autopoll_mutex);
			up(&pdx->autopoll_mutex);
			return;
		}
		//....ExAcquireFastMutex (&pdx->autopoll_mutex);
		if((pdx->autospollers <= 0) || (!(pdx->status & SRQI)))
		{
			up(&pdx->autopoll_mutex);
			//ExReleaseFastMutex (&pdx->autopoll_mutex);
			return;
		}
		retval = autopoll_all(pdx);
		if(retval <= 0)
		{
			//yuan modify 10/12/05
			pdx->stuck_srq = 1;
			set_bit(SRQI_NUM, &pdx->status);
		}
		//ExReleaseFastMutex (&pdx->autopoll_mutex);
		up(&pdx->autopoll_mutex);
	}	


unsigned int num_gpib_events( const gpib_event_queue_t *queue )
{
	return queue->num_events;
}

gpib_descriptor_t* get_descriptor( const gpib_file_private_t *file_priv,
	int handle )
{
	if( handle < 0 || handle >= GPIB_MAX_NUM_DESCRIPTORS )
	{
		//printk( "invalid handle %i\n", handle );
		return NULL;
	}

	return file_priv->descriptors[ handle ];

}

void init_gpib_status_queue( gpib_status_queue_t *device )
{
	INIT_LIST_HEAD( &device->list );
        INIT_LIST_HEAD( &device->status_bytes );
	device->num_status_bytes = 0;
	device->reference_count = 0;
	device->dropped_byte = 0;
	//device->r_end = 0;
	//device->cur_pos = 0;
	//device->buffer = NULL;
}

static int insert_one_device( struct list_head *head, unsigned int pad, int sad, unsigned int usec_timeout, PFIFO_BUF *addr )
{
	struct list_head *list_ptr;
	gpib_status_queue_t *device;

	for( list_ptr = head->next; list_ptr != head; list_ptr = list_ptr->next )
//	for( list_ptr = head->Flink; list_ptr != head; list_ptr = list_ptr->Flink )
	{
		device = list_entry( list_ptr, gpib_status_queue_t, list );
		if( addr_cmp( device->pad, device->sad, pad, sad ) )
		{
			//GPIB_DPRINTK( "incrementing open count for pad %i, sad %i\n",
			//	device->pad, device->sad );
			*addr = device->fifo_buf;
			device->reference_count++;
			return 0;
		}
	}

	/* create a new device */
	device = kmalloc(sizeof( gpib_status_queue_t ), GFP_ATOMIC );
	if( device == NULL )
		return -2;//-ENOMEM;
	init_gpib_status_queue( device );
	device->pad = pad;
	device->sad = sad;
	device->usec_timeout = usec_timeout;
	device->reference_count = 1;

	device->fifo_buf = kmalloc(sizeof(FIFO_BUF), GFP_ATOMIC );
	if( device->fifo_buf == NULL )
		return -2;//-ENOMEM;
	device->fifo_buf->r_end = 0;
	device->fifo_buf->cur_pos = 0;
	device->fifo_buf->buffer = NULL;
	device->fifo_buf->buffer = kmalloc(ad9607_fifo_size+2, GFP_ATOMIC );
	if( device->fifo_buf->buffer == NULL )
		return -2;//-ENOMEM;
	list_add( &device->list, head );
	
        *addr = device->fifo_buf;
	return 0;
}

int remove_open_device_ex( struct list_head *head, unsigned int pad, int sad, unsigned int count )
{
	gpib_status_queue_t *device;
	struct list_head *list_ptr;

	for( list_ptr = head->next; list_ptr != head; list_ptr = list_ptr->next )
	//for( list_ptr = head->Flink; list_ptr != head; list_ptr = list_ptr->Flink )
	{
		device = list_entry( list_ptr, gpib_status_queue_t, list );
		if( addr_cmp( device->pad, device->sad, pad, sad ) )
		{
			if( count > device->reference_count )
			{
				//yuan add 07/22/09
				count = device->reference_count;
				//return -1;//-EINVAL;
			}
			device->reference_count -= count;
			if( device->reference_count == 0 )
			{
				//GPIB_DPRINTK( "closing pad %i, sad %i\n",
				//	device->pad, device->sad );
				//list_del( list_ptr );
				if(device->fifo_buf) {
					if(device->fifo_buf->buffer) 
						kfree( device->fifo_buf->buffer );
						//AdlDeleteNpp( device->fifo_buf->buffer );
					//AdlDeleteNpp( device->fifo_buf );
					kfree ( device->fifo_buf );
				}
				device->fifo_buf = NULL;
				list_del( list_ptr );				
				kfree( device );
				//AdlDeleteNpp( device );
			}
			return 0;
		}
	}
	//return -1;//-EINVAL;
	return 0;
}

int ibpad( PU348A_DEVEXT board, unsigned int addr )
{
	if ( addr > 30 )
	{
		//printk("invalid primary address %u\n", addr );
		return -1;
	}else
	{
		board->pad = addr;
		if( board->online )
			adgpib_primary_address(board, board->pad);
	}
	return 0;
}


int ibsad( PU348A_DEVEXT board, int addr )
{
	if( addr > 30 )
	{
		//printk("invalid secondary address %i\n", addr);
		return -1;
	}else
	{
		board->sad = addr;
		if( board->online )
		{
			if( board->sad >= 0 )
			{
				adgpib_secondary_address( board, board->sad, 1 );
			}else
			{
				adgpib_secondary_address( board, 0, 0 );
			}
		}
	}
	return 0;
}


int ibeos( U348A_DEVEXT *board, int eos, int eosflags )
{
	if( eosflags & ~EOS_MASK )
	{
		//printk( "invalid EOS mode\n" );
		return -1;//-EINVAL;
	}else
	{
		if( eosflags & REOS )
		{
			adgpib_eos_enable( board, eos, eosflags & BIN );
		}else
			adgpib_eos_disable( board );
	}
	return 0;
}

unsigned int _t1_delay( U348A_DEVEXT *priv, unsigned int nano_sec )
{
	unsigned int retval;

	if( nano_sec <= 500 )
	{
		priv->auxb_bits |= HR_TRI;
		retval = 500;
	}else
	{
		priv->auxb_bits &= ~HR_TRI;
		retval = 2000;
	}
	write_byte( priv, priv->auxb_bits, AUXMR );

	return retval;
}

unsigned int adgpib_t1_delay( U348A_DEVEXT *priv, unsigned int nano_sec )
{
	unsigned int retval;

	retval = _t1_delay( priv, nano_sec );
	if( nano_sec <= 350 )
		{	
			priv->auxi_bits |= HR_USTD;
	   		write_byte(priv, priv->auxi_bits, AUXMR);
	   		retval = 350;
		}else {
			priv->auxi_bits &= ~HR_USTD;
	   		write_byte(priv, priv->auxi_bits, AUXMR);			
		}
	
	return retval;
}

//ioctl
int read_kioctl( gpib_file_private_t *file_priv, U348A_DEVEXT *board,
	unsigned long arg)
{
	read_write_ioctl_t read_cmd;
	uint8_t *userbuf;
	unsigned long remain;
	int end_flag = 0;
	int retval=0;
	ssize_t read_ret = 0;
	gpib_descriptor_t *desc;
	size_t nbytes;
  uint8_t adsr_bits=0;
	
	retval = copy_from_user(&read_cmd, (void*) arg, sizeof(read_cmd));
	if (retval)
		return -EFAULT;

	desc = get_descriptor( file_priv, read_cmd.handle );
	if( desc == NULL ) return -EINVAL;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0))
	if(!access_ok(VERIFY_WRITE, read_cmd.buffer, read_cmd.count))
#else
	if(!access_ok(read_cmd.buffer, read_cmd.count))
#endif 
#if 0
	if(!access_ok(VERIFY_WRITE, read_cmd.buffer, read_cmd.count))
#endif
		return -EFAULT;

	if((desc->io_in_progress == 1) || (board->io_in_progress == 1) ) {
		//yuan add 01/17/08
		///KeSetEvent(&(board->dpcEvt[1]), 0, FALSE);
		return -1;
	}
	board->io_in_progress = desc->io_in_progress = 1;
  if(desc->abort) {
		board->io_in_progress = desc->io_in_progress = 0;
		//yuan add 01/17/08
//		KeSetEvent(&(board->dpcEvt[1]), 0, FALSE);
		desc->abort = 0;
			//yuan add for address setting flag 10/06/05
		//board->stuck_srq = 0;
		return -ETIMEDOUT;
  }
  desc->abort = 0;

	userbuf = read_cmd.buffer;
	remain = read_cmd.count;

	if((!userbuf) || (!remain)) {
		board->io_in_progress = desc->io_in_progress = 0;
		//yuan add 01/17/08
//		KeSetEvent(&(board->dpcEvt[1]), 0, FALSE);
			//yuan add for address setting flag 10/06/05
		//board->stuck_srq = 0;
		return 0;
	}
	
	///ExAcquireFastMutex (&board->autopoll_mutex);
	retval = down_interruptible(&board->autopoll_mutex);
	board->usec_timeout = read_cmd.usec_timeout;
	if(board->buffer_length < read_cmd.count) {
		unsigned char *tmpB;
		tmpB = kmalloc(read_cmd.count, GFP_KERNEL );
		if(tmpB) {
			kfree( board->buffer );
			board->buffer_length = read_cmd.count;
			board->buffer = tmpB;
		}
	}
	while(remain > 0 && (end_flag == 0))
	{
		nbytes = 0;
		memset(board->buffer, '\0', board->buffer_length);
		board->dwDevRA = 0;
		board->dwDevRB = (board->buffer_length < remain) ? board->buffer_length : remain;
		read_ret = ibrd(board, desc, board->buffer, board->dwDevRB, &end_flag);
//printk("ibrd: %d\n", read_ret);
		if(read_ret == 0) break;
		if(read_ret < 0)
		{
		 if(board->hs_mode_bits & HS_RX_ENABLE) {
			if(board->dwDevRA) {
				//KdPrint(("before copy %d\n", ret));
				retval = copy_to_user(userbuf, board->buffer, board->dwDevRA);
				if(retval)
				{
					retval = -EFAULT;
					break;
				}
				//memcpy(userbuf, board->buffer, ret);
				//printk("after copy\n");
			}
			remain -= board->dwDevRA;
			read_cmd.count -= remain;
		 }
		 board->dwDevRA = 0;
	
		 board->io_in_progress = desc->io_in_progress = 0;
		 desc->abort = 0;
		 board->dwDevRB = 0;
		 up(&board->autopoll_mutex);
		 //ExReleaseFastMutex (&board->autopoll_mutex);
			//KeSetEvent(&board->evKill, 0, FALSE);  
		 read_cmd.handle = read_ret;
		 read_cmd.end = end_flag;
		 retval = copy_to_user((void*) arg, &read_cmd, sizeof(read_cmd));
		 return 0;//ret;
		}
		if( read_ret == 0 ) {
			break;
		}
		//printk("before copy\n");
		retval = copy_to_user(userbuf, board->buffer, board->dwDevRA);
		if(retval)
		{
			retval = -EFAULT;
			break;
		}
		remain -= read_ret;
		userbuf += read_ret;
	  //printk("after copy\n");
	} //while
//end_f:
	read_cmd.count -= remain;   
	//KdPrint(("line %x\n", iblines(board)));
	if(end_flag) {
		 board->hs_mode_bits &= ~(HS_THRES_INT_EN);
		 board->ThresHoldCnt = ad9607_fifo_size / 2;
		 clear_bit(RECEIVED_END_BN, &board->state);
		 clear_bit(READ_READY_BN, &board->state);
	 	 board->dwDevRB = 0;
	} //end_flag = 1 
	board->io_in_progress = desc->io_in_progress = 0;

	read_cmd.end = end_flag;
	read_cmd.handle = 0;//board->state;
	//printk("return end %d\n", read_cmd.end);
	board->dwDevRA = 0;
	board->dwDevRB = 0;
	desc->abort = 0;
	up(&board->autopoll_mutex);
	//ExReleaseFastMutex (&board->autopoll_mutex);
	//KeSetEvent(&board->evKill, 0, FALSE);  
	retval = copy_to_user((void*) arg, &read_cmd, sizeof(read_cmd));    		    
	///wake_up_interruptible( &board->wait );
	if(retval)  {
		printk("copy to user failed\n");
		return -EFAULT;
	}
	return read_ret;
}

int command_kioctl( gpib_file_private_t *file_priv,
	U348A_DEVEXT *board, unsigned long arg)
{
	read_write_ioctl_t cmd;
	uint8_t *userbuf;
	unsigned long remain;
	int retval, taddr=0;
	gpib_descriptor_t *desc;
	signed char spd = 0, io=0;

	retval = copy_from_user(&cmd, (void*) arg, sizeof(cmd));
	if( retval )
	{
		printk("cmd copy from user failed\n");
		return -EFAULT;
	}

	desc = get_descriptor( file_priv, cmd.handle );
	if( desc == NULL ) return -EINVAL;
	if( (desc->io_in_progress == 1)  || (board->io_in_progress == 1) ) return -EINVAL;

	board->io_in_progress = desc->io_in_progress = 1;
	if(desc->abort) {
		board->io_in_progress = desc->io_in_progress = 0;
		desc->abort = 0;
		return -ETIMEDOUT;
	}
  desc->abort = 0;
	if(down_interruptible(&board->autopoll_mutex))
		//return -ERESTARTSYS;
	printk("cmd acquire mutex failed\n");
	
	//write_byte( board, AUX_SREN, AUXMR);

	io = ((cmd.end>>16) & 0x01) | ((cmd.end>>20) & 0x01);
	if(io)
		board->locking_pid = current->pid;//IoGetCurrentProcess();
	//printk("io %x %x\n", io, cmd.end);
      //yuan add 05/26/05
	if(((cmd.end>>16) & 0x01) == 1) {
		taddr = cmd.end & 0xffff;
	} else {
		taddr = board->pad & 0xff;
		if(board->sad >= 0)
			taddr |= (board->sad|0x60) << 8 ;
	}
	//yuan add 08/22/06
	if(((cmd.end>>16) & 0x04) == 4) {
			//SPE
			board->spe = 1;
	}
	if(((cmd.end>>16) & 0x08) == 8) {
			//SPE
			board->spe = 0;
			spd = 1;
	}
	
	if(((cmd.end>>16) & 0x01) == 1) {
		if(board->cur_taddr == taddr) {
			board->io_in_progress = desc->io_in_progress = 0;
			up(&board->autopoll_mutex);
			return 0;
		}
	} 

	board->usec_timeout = cmd.usec_timeout;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0))
	if(!access_ok(VERIFY_READ, cmd.buffer, cmd.count))
#else
	if(!access_ok(cmd.buffer, cmd.count))
#endif 
#if 0
	if(!access_ok(VERIFY_READ, cmd.buffer, cmd.count))
#endif
	{
		up(&board->autopoll_mutex);
		printk("access failed\n");
		return -EFAULT;
	}
	memset(board->buffer, '\0', board->buffer_length);

	userbuf = cmd.buffer;
	remain = cmd.count;

	//desc->io_in_progress = 1;
	while( remain > 0 )
	{
		retval = copy_from_user(board->buffer, userbuf, (board->buffer_length < remain) ?
			board->buffer_length : remain );
		if(retval) retval = -EFAULT;
		else
			retval = ibcmd(board, desc, board->buffer, (board->buffer_length < remain) ?
				board->buffer_length : remain );
		if(retval < 0)
		{
		 printk("cmd failed: %d\n", retval);
			board->io_in_progress  = desc->io_in_progress = 0;
			desc->abort = 0;
			if(io)
				board->locking_pid = 0;
		  up(&board->autopoll_mutex);
			//wake_up_interruptible( &board->wait );
			return retval;
		}
		if( retval == 0 ) break;
		remain -= retval;
		userbuf += retval;
	}

	cmd.count -= remain;
	if(desc->abort) {
	  board->io_in_progress = desc->io_in_progress = 0;
	  desc->abort = 0;
	  up(&board->autopoll_mutex);	
	//ExReleaseFastMutex (&board->autopoll_mutex);
	  printk("cmd abort: %d\n", retval);
	  return -ETIMEDOUT;
	}
		
	board->io_in_progress = desc->io_in_progress = 0;
	desc->abort = 0;
  board->cur_taddr = taddr;
	retval = copy_to_user((void*) arg, &cmd, sizeof(cmd));
	up(&board->autopoll_mutex);	
	//printk("cmd end\n");
	return 0;
}

int write_kioctl(gpib_file_private_t *file_priv, U348A_DEVEXT *board,
	unsigned long arg)
{
	read_write_ioctl_t write_cmd;
	uint8_t *userbuf;
	unsigned long remain;
	int retval = 0;
	int fault;
	gpib_descriptor_t *desc;

	fault = copy_from_user(&write_cmd, (void*) arg, sizeof(write_cmd));
	if(fault)
		return -EFAULT;

	desc = get_descriptor( file_priv, write_cmd.handle );
	if( desc == NULL ) return -EINVAL;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0))
	if(!access_ok(VERIFY_READ, write_cmd.buffer, write_cmd.count))
#else
	if(!access_ok(write_cmd.buffer, write_cmd.count))
#endif 	
#if 0
	if(!access_ok(VERIFY_READ, write_cmd.buffer, write_cmd.count))
#endif
	{
	printk("wrt access_ok failed\n");
		return -EFAULT;
	}
	userbuf = write_cmd.buffer;
	remain = write_cmd.count;
	if((!userbuf) || (!remain)) {
		board->io_in_progress = desc->io_in_progress = 0;
		return 0;
	}

	if(down_interruptible(&board->autopoll_mutex))
		//return -ERESTARTSYS;
		printk("wrt acquire mutex failed\n");
	board->usec_timeout = write_cmd.usec_timeout;
	if(board->buffer_length < write_cmd.count) {
		unsigned char *tmpB;
		tmpB = kmalloc(write_cmd.count, GFP_KERNEL );
		if(tmpB) {
			kfree( board->buffer );
			board->buffer_length = write_cmd.count;
			board->buffer = tmpB;
		}
	}
while( remain > 0 )
	{
		int send_eoi;
		ssize_t bytes_written = 0;
		if((remain <= board->buffer_length) && write_cmd.end)
			send_eoi = 1;
		else
			send_eoi = 0; 
		fault = copy_from_user(board->buffer, userbuf, (board->buffer_length < remain) ?
			board->buffer_length : remain );
		if(fault)
		{
			retval = -EFAULT;
			////break;
		}
	  board->dwDevRA = 0;
	  board->dwDevRB = 0;
		bytes_written = ibwrt(board, desc, board->buffer, (board->buffer_length < remain) ?
			board->buffer_length : remain, send_eoi);		
		if(bytes_written < 0)
		{
     printk("ibwrt error %zd %d\n", bytes_written, write_cmd.count);
		 board->io_in_progress = desc->io_in_progress = 0;	
		 fault = copy_to_user((void*) arg, &write_cmd, sizeof(write_cmd));
	 ///KeSetEvent(&(board->dpcEvt[1]), 0, FALSE);
	   board->dwDevRA = 0;
	   board->dwDevRB = 0;
		 desc->abort = 0;
		 up(&board->autopoll_mutex);	
		 return bytes_written;//retval;
		}
		remain -= bytes_written;
		userbuf += bytes_written;

	}
	write_cmd.count -= remain;
	if(remain == 0)
	{
		retval = 0;
	}
  board->io_in_progress = desc->io_in_progress = 0;
	board->dwDevRA = 0;
	board->dwDevRB = 0;
	desc->abort = 0;
	fault = copy_to_user((void*) arg, &write_cmd, sizeof(write_cmd));
	//desc->io_in_progress = 0;
	up(&board->autopoll_mutex);					
	//printk("write finish\n");
	return 0;//retval;
}

int status_bytes_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	gpib_status_queue_t *device;
	spoll_bytes_ioctl_t cmd;
	int retval;

	retval = copy_from_user( &cmd, (void *) arg, sizeof( cmd ) );
	if( retval )
		return -EFAULT;

	device = get_spoll_status_queue( board, cmd.pad, cmd.sad );
	if( device == NULL )
		cmd.num_bytes = 0;
	else
		cmd.num_bytes = num_status_bytes( device );

	retval = copy_to_user( (void *) arg, &cmd, sizeof( cmd ) );
	if( retval )
		return -EFAULT;

	return 0;
}

int open_dev_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	open_dev_ioctl_t open_dev_cmd;
	int retval;
	gpib_file_private_t *file_priv = board->private_data;
	int i;

	retval = copy_from_user( &open_dev_cmd, ( void* ) arg, sizeof( open_dev_cmd ) );
	if (retval)
		return -EFAULT;

	//if(down_interruptible(&file_priv->descriptors_mutex))
	//{
	//	return -ERESTARTSYS;
	//}
	for( i = 0; i < GPIB_MAX_NUM_DESCRIPTORS; i++ )
		if( file_priv->descriptors[ i ] == NULL ) break;
	if( i == GPIB_MAX_NUM_DESCRIPTORS )
		return -ERANGE;
	file_priv->descriptors[ i ] = kmalloc( sizeof( gpib_descriptor_t ), GFP_KERNEL );
	if( file_priv->descriptors[ i ] == NULL )
	{
//		up(&file_priv->descriptors_mutex);
		return -ENOMEM;
	}
	init_gpib_descriptor( file_priv->descriptors[ i ] );

	file_priv->descriptors[ i ]->pad = open_dev_cmd.pad;
	file_priv->descriptors[ i ]->sad = open_dev_cmd.sad;
	file_priv->descriptors[ i ]->is_board = open_dev_cmd.is_board;
	//up(&file_priv->descriptors_mutex);
	retval = insert_one_device( &board->device_list, open_dev_cmd.pad, open_dev_cmd.sad, open_dev_cmd.usec_timeout, &(file_priv->descriptors[ i ]->fifo_buf) );
	if( retval < 0 )
		return retval;

	board->stuck_srq = 0;
	open_dev_cmd.handle = i;
	retval = copy_to_user( ( void* ) arg, &open_dev_cmd, sizeof( open_dev_cmd ) );
	if (retval)
		return -EFAULT;

	return 0;
}

int close_dev_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	close_dev_ioctl_t cmd;
	gpib_file_private_t *file_priv = board->private_data;
	int retval;

	retval = copy_from_user( &cmd, ( void* ) arg, sizeof( cmd ) );
	if (retval)
		return -EFAULT;

	if( cmd.handle >= GPIB_MAX_NUM_DESCRIPTORS ) return -EINVAL;
	if( file_priv->descriptors[ cmd.handle ] == NULL ) return -EINVAL;

	retval = remove_open_device( &board->device_list, file_priv->descriptors[ cmd.handle ]->pad,
		file_priv->descriptors[ cmd.handle ]->sad );
	if( retval < 0 ) return retval;

	kfree( file_priv->descriptors[ cmd.handle ] );
	file_priv->descriptors[ cmd.handle ] = NULL;

	return 0;
}

int serial_poll_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	serial_poll_ioctl_t serial_cmd;
	int retval;

	retval = copy_from_user( &serial_cmd, ( void* ) arg, sizeof( serial_cmd ) );
	if( retval )
		return -EFAULT;

	retval = get_spoll_status_byte( board, serial_cmd.pad, serial_cmd.sad, board->usec_timeout,
		&serial_cmd.status_byte );
	if( retval < 0 )
		return retval;

	retval = copy_to_user( ( void * ) arg, &serial_cmd, sizeof( serial_cmd ) );
	if( retval )
		return -EFAULT;

	return 0;
}

//[0]:int1
//[1]:int2
//[2]: adsr
//[3]: line 
int staus_reads(PU348A_DEVEXT priv, uint8_t *rdbuf)
{
int retval =0;
	retval = UDServiceFire(priv, VR_GPIB_STATUS, 0, 0, 4, rdbuf, 1);
	if (retval<0) {
		//error
		retval = -1;
	} else
		retval = 0;
	return retval;
}

unsigned int update_status_nolock_2( PU348A_DEVEXT priv, uint8_t address_status_bits0)
{
	int address_status_bits = address_status_bits0;
 
	if(priv == NULL) return 0;

	if(address_status_bits & HR_CIC)
		set_bit(CIC_NUM, &priv->status);
	else
		clear_bit(CIC_NUM, &priv->status);
	// check for talker/listener addressed
	if(address_status_bits & HR_TA)
	{
		set_bit(TACS_NUM, &priv->status);
	}else
		clear_bit(TACS_NUM, &priv->status);
	if(address_status_bits & HR_LA)
	{
		set_bit(LACS_NUM, &priv->status);
	}else
		clear_bit(LACS_NUM, &priv->status);
	if(address_status_bits & HR_NATN)
	{
		clear_bit(ATN_NUM, &priv->status);
	}else
	{
		set_bit(ATN_NUM, &priv->status);
	}

//	GPIB_DPRINTK( "status 0x%x, state 0x%x\n", board->status, priv->state );

	/* we rely on the interrupt handler to set the
	 * rest of the status bits */

	return priv->status;
}

unsigned int u348a_update_status_2(PU348A_DEVEXT board, unsigned int clear_mask, uint8_t address_status_bits0)
{
unsigned int retval=0;
	retval = update_status_nolock_2( board, address_status_bits0);
	board->status &= ~clear_mask;

	return retval;
}

short iblines_2( PU348A_DEVEXT board, uint8_t bsr_bits0)
{
	int status = ValidALL;
	int bsr_bits = bsr_bits0;

	    if( bsr_bits & BSR_REN_BIT_9607 )
		status |= BusREN;
	    if( bsr_bits & BSR_IFC_BIT_9607 )
		status |= BusIFC;
	    if( bsr_bits & BSR_SRQ_BIT_9607 )
		status |= BusSRQ;
	    if( bsr_bits & BSR_EOI_BIT_9607 )
		status |= BusEOI;
	    if( bsr_bits & BSR_NRFD_BIT_9607 )
		status |= BusNRFD;
	    if( bsr_bits & BSR_NDAC_BIT_9607 )
		status |= BusNDAC;
	    if( bsr_bits & BSR_DAV_BIT_9607 )
		status |= BusDAV;
	    if( bsr_bits & BSR_ATN_BIT_9607 )
		status |= BusATN;	

	return ((short) status);
}

int inquire_ibstatus( U348A_DEVEXT *board, const gpib_status_queue_t *device,
	int clear_mask, int set_mask, gpib_descriptor_t *desc )
{
	int status = 0, ret=0;
	short line_status=0;
	unsigned char rdbuf[4];

	if( board->private_data )
	{
	    if( device && (set_mask & RQS)) 
		  board->stuck_srq = 0;
	    staus_reads(board, rdbuf);
	    //printk("status: %x %x %x %x\n", rdbuf[0], rdbuf[1], rdbuf[2], rdbuf[3] );	    	
	    adgpib_isr1_status( board, rdbuf[0] );
	    adgpib_isr2_status( board, rdbuf[1] );

	    status = u348a_update_status_2( board, clear_mask, rdbuf[2]);		  
	    //aaa status = adgpib_update_status( board, clear_mask );
	    status &= ~TIMO;
		/* get line status (SRQI) status */
		if(desc && (! desc->io_in_progress) ) 
		{
		 line_status = iblines_2(board, rdbuf[3]);//////iblines(board);
		 //aaa line_status = iblines(board);
		 if((line_status & ValidSRQ))
		 {
		//printk("valid SRQ: %x %x %d %d\n", line_status, line_status & BusSRQ, clear_mask & RQS, board->locking_pid);
			if((line_status & BusSRQ))
			{
//printk("valid SRQ: %x %x %d %d\n", line_status, line_status & BusSRQ, clear_mask & RQS, board->locking_pid);
				status |= SRQI;;
				board->status |= SRQI;
				if((!board->stuck_srq) && device && (!(clear_mask & RQS)) && (!board->locking_pid)) {
					ret = down_interruptible(&board->autopoll_mutex);
					//ExAcquireFastMutex (&board->autopoll_mutex);
					PollingRoutine(board);
				}
			}else
			{
				status &= ~SRQI;
				//status &= ~RQS;
			}
		 }
		}
	}
	if( device )
		if( num_status_bytes( device ) ) status |= RQS;

	if( desc )
	{
		if( desc->io_in_progress )
			status &= ~CMPL;
		else
			status |= CMPL;			
	}
	//yuan add 02/21/08	
	status &= ~END; 
	//yuan add for TCT 07/10/09
	/***if(!board->master && ((status & CIC) != 0)) {
		board->master = 1;
	}***/	
	return status;
}

int wait_kioctl( gpib_file_private_t *file_priv, U348A_DEVEXT *board,
	unsigned long arg )
{
	wait_ioctl_t wait_cmd;
	int retval;
	gpib_descriptor_t *desc;

	retval = copy_from_user( &wait_cmd, ( void * ) arg, sizeof( wait_cmd ) );
	if( retval )
		return -EFAULT;

	desc = get_descriptor( file_priv, wait_cmd.handle );
	if( desc == NULL ) return -EINVAL;
	{
		gpib_status_queue_t *status_queue;
		if( desc->is_board ) status_queue = (gpib_status_queue_t *) NULL;
		else status_queue = get_spoll_status_queue( board, desc->pad, desc->sad );
		//yuan modify 02/28/07
		wait_cmd.ibsta = inquire_ibstatus( board, status_queue, wait_cmd.clear_mask, wait_cmd.wait_mask, desc );

	}
	//retval = ibwait( board, wait_cmd.wait_mask, wait_cmd.clear_mask,
	//	wait_cmd.set_mask, &wait_cmd.ibsta, wait_cmd.usec_timeout, desc );
	if( retval < 0 ) return retval;

	retval = copy_to_user( ( void * ) arg, &wait_cmd, sizeof( wait_cmd ) );
	if( retval )
		return -EFAULT;

	return 0;
}


int sta_kioctl( PU348A_DEVEXT board, unsigned long arg )
{
	wait_ioctl_t wait_cmd;
	int retval;

	retval = copy_from_user( &wait_cmd, ( void * ) arg, sizeof( wait_cmd ) );
	if( retval )
		return -EFAULT;

	wait_cmd.ibsta = read_byte(board, ADSR);
	retval = copy_to_user( ( void * ) arg, &wait_cmd, sizeof( wait_cmd ) );
	if( retval )
		return -EFAULT;
	return 0;
}

int parallel_poll_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	ppoll_ioctl_t ppoll_cmd;
	uint8_t poll_byte;
	int retval;

	retval = copy_from_user( &ppoll_cmd, ( void * ) arg, sizeof( ppoll_cmd ) );
	if( retval )
		return -EFAULT;
	board->ppoll_usec_timeout = ppoll_cmd.ppoll_usec_timeout;
	retval = ibrpp( board, &poll_byte );
	if( retval < 0 )
		return retval;
	ppoll_cmd.poll_byte = poll_byte;
	retval = copy_to_user( ( void * ) arg, &ppoll_cmd, sizeof( ppoll_cmd ) );
	if( retval )
		return -EFAULT;

	return 0;
}

int online_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
  online_ioctl_t online_cmd;
  int retval=0;
  uint8_t rwbuf[128];
  uint8_t rwcnt = 0;

  retval = copy_from_user( &online_cmd, ( void * ) arg, sizeof( online_cmd ) );
  if(retval)
	return -EFAULT;
  rwcnt = 0;

	if( online_cmd.online == 1)
		//return ibonline( board );
		board->online = 1;
	if( online_cmd.online == 2) {//reset
		//return ibonline( board );
	  u348a_board_reset( board );
	  board->t1_nano_sec = adgpib_t1_delay( board, 500 );
    memset(rwbuf, 0, 128);
    rwbuf[rwcnt++] = AUXMR;
    rwbuf[rwcnt++] = ICR | 0;	    	 
	  board->dwDevRA = board->dwDevRB = 0;
    board->hs_mode_bits = HS_TX_ENABLE|HS_RX_ENABLE;//|HS_SYS_CONTROL;
    board->cur_taddr = UNT;//SetOnLoad
    ioport_Mult_Read_write(board, 0, 0, rwcnt, rwbuf);
	  u348a_board_online( board );
	  board->exclusive = 0;
  } else if(online_cmd.online == 3) {
	   rwcnt = 0;
		  rwbuf[rwcnt++] = ADR;
		  rwbuf[rwcnt++] = 0x60;
		  rwbuf[rwcnt++] = ADR;
		  rwbuf[rwcnt++] = 0xE0;
		  rwbuf[rwcnt++] = ADMR;
		  rwbuf[rwcnt++] = 0x70;
		  ioport_Mult_Read_write(board, 0, 0, rwcnt, rwbuf);
		  board->exclusive = 1;
	} else if(!online_cmd.online) {
		board->online = 0; //and then detach		
		board->exclusive = 0;
	}

	return retval;
}

int remote_enable_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	int enable;
	int retval;

	retval = copy_from_user( &enable, ( void * ) arg, sizeof( enable ) );
	if( retval )
		return -EFAULT;

	return ibsre( board, enable );
}

int take_control_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	int synchronous;
	int retval;

	retval = copy_from_user( &synchronous, ( void * ) arg, sizeof( synchronous ) );
	if( retval )
		return -EFAULT;

	return ibcac( board, synchronous );
}

int line_status_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	short lines;
	int retval;

	lines = iblines( board);
	retval = copy_to_user( ( void * ) arg, &lines, sizeof( lines ) );
	if( retval )
		return -EFAULT;

	return 0;
}

int pad_kioctl( U348A_DEVEXT *board, gpib_file_private_t *file_priv,
	unsigned long arg )
{
	pad_ioctl_t cmd;
	int retval;
	gpib_descriptor_t *desc;

	retval = copy_from_user( &cmd, ( void * ) arg, sizeof( cmd ) );
	if( retval )
		return -EFAULT;

	desc = get_descriptor( file_priv, cmd.handle );
	if( desc == NULL )
		return -EINVAL;

	if( desc->is_board )
	{
		retval = ibpad( board, cmd.pad );
		if( retval < 0 ) return retval;
	}else
	{
		retval = remove_open_device( &board->device_list, desc->pad, desc->sad );
		if( retval < 0 )
			return retval;

		desc->pad = cmd.pad;

		retval = insert_one_device( &board->device_list, desc->pad, desc->sad, desc->usec_timeout, &(desc->fifo_buf) );
		if( retval < 0 )
			return retval;
	}

	return 0;
}

int sad_kioctl( U348A_DEVEXT *board, gpib_file_private_t *file_priv,
	unsigned long arg )
{
	sad_ioctl_t cmd;
	int retval;
	gpib_descriptor_t *desc;

	retval = copy_from_user( &cmd, ( void * ) arg, sizeof( cmd ) );
	if( retval )
		return -EFAULT;

	desc = get_descriptor( file_priv, cmd.handle );
	if( desc == NULL )
		return -EINVAL;

	if( desc->is_board )
	{
		retval = ibsad( board, cmd.sad );
		if( retval < 0 ) return retval;
	}else
	{
		retval = remove_open_device( &board->device_list, desc->pad, desc->sad );
		if( retval < 0 )
			return retval;

		desc->sad = cmd.sad;

		retval = insert_one_device( &board->device_list, desc->pad, desc->sad, desc->usec_timeout, &(desc->fifo_buf) );
		if( retval < 0 )
			return retval;
	}
	return 0;
}

int eos_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	eos_ioctl_t eos_cmd;
	int retval;

	retval = copy_from_user( &eos_cmd, ( void * ) arg, sizeof( eos_cmd ) );
	if( retval )
		return -EFAULT;

	return ibeos( board, eos_cmd.eos, eos_cmd.eos_flags );
}

int request_service_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	uint8_t status_byte;
	int retval;

	retval = copy_from_user( &status_byte, ( void * ) arg, sizeof( status_byte ) );
	if( retval )
		return -EFAULT;

	return ibrsv( board, status_byte );
}

int autospoll_kioctl(U348A_DEVEXT *board, unsigned long arg)
{
	autospoll_ioctl_t enable;
	int retval;

	retval = copy_from_user( &enable, ( void * ) arg, sizeof( enable ) );
	if(retval)
		return -EFAULT;

	if(down_interruptible(&board->autopoll_mutex))
	{
		return -ERESTARTSYS;
	}
	if(enable)
		board->autospollers++;
	else
	{
		if(board->autospollers <= 0)
		{
			board->autospollers = 0;
			retval = 0;
		}else
		{
			board->autospollers--;
			retval = 0;
		}
	}
	up( &board->autopoll_mutex );
	return retval;
}

int timeout_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	unsigned int timeout;
	int retval;

	retval = copy_from_user( &timeout, ( void * ) arg, sizeof( timeout ) );
	if( retval )
		return -EFAULT;

	board->usec_timeout = timeout;
	//printk( "timeout set to %i usec\n", timeout );

	return 0;
}

int ppc_kioctl( U348A_DEVEXT *board, unsigned long arg)
{
	ppoll_config_ioctl_t cmd;
	int retval;

	retval = copy_from_user( &cmd, ( void * ) arg, sizeof( cmd ) );
	if( retval )
		return -EFAULT;
	
	if( cmd.set_ist )
	{
		board->ist = 1;
		adgpib_ibist( board, board->ist );
	}else if( cmd.clear_ist )
	{
		board->ist = 0;
		adgpib_ibist( board, board->ist );
	}
#if 0	
	else if(!cmd.config ) 
	{		
		if((board->auxi_bits & HR_PP2)) 
		{
				board->auxi_bits &= ~HR_PP2;
				write_byte(board, board->auxi_bits, AUXMR);
		}
	}
#endif
	if( cmd.config )
	{
#if 0		
		if(!(board->auxi_bits & HR_PP2))
		{
				board->auxi_bits |= HR_PP2;
				write_byte(board, board->auxi_bits, AUXMR);
		}
#endif		
		if(!(cmd.config & 0x60))
			return 0;
		retval = ibppc( board, cmd.config );
		if( retval < 0 ) return retval;
	}

	return 0;
}

int query_board_rsv_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	int status;
	int retval;

	status = read_byte( board, SPSR );
	retval = copy_to_user( ( void * ) arg, &status, sizeof( status ) );
	if( retval )
		return -EFAULT;

	return 0;
}

int board_info_kioctl( const U348A_DEVEXT *board, unsigned long arg)
{
	board_info_ioctl_t info;
	int retval;
	info.pad = board->pad;
	info.sad = board->sad;
	info.parallel_poll_configuration = board->parallel_poll_configuration;
	info.is_system_controller = board->master;
	if( board->autospollers )
		info.autopolling = 1;
	else
		info.autopolling = 0;
	info.t1_delay = board->t1_nano_sec;
	info.ist = board->ist;
	retval = copy_to_user( ( void * ) arg, &info, sizeof( info ) );
	if( retval )
		return -EFAULT;
	return 0;
}

int interface_clear_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	unsigned int usec_duration;
	int retval;
	
	retval = copy_from_user( &usec_duration, ( void * ) arg, sizeof( usec_duration ) );
	if( retval )
		return -EFAULT;

	return ibsic( board, usec_duration );
}

int request_system_control_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	rsc_ioctl_t request_control;
	int retval;

	retval = copy_from_user( &request_control, ( void * ) arg, sizeof( request_control ) );
	if( retval ) return -EFAULT;

	ibrsc( board, request_control );

	return 0;
}

int t1_delay_kioctl( U348A_DEVEXT *board, unsigned long arg )
{
	t1_delay_ioctl_t cmd;
	unsigned int delay;
	int retval;

	retval = copy_from_user( &cmd, ( void * ) arg, sizeof( cmd ) );
	if( retval ) return -EFAULT;
	delay = cmd;
	board->t1_nano_sec = adgpib_t1_delay( board, delay );

	return 0;
}

int ibstatus( PU348A_DEVEXT board )
{
	return inquire_ibstatus( board, NULL, 0, 0, NULL);
}

