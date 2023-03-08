/*
 * USB Skeleton driver - 2.0
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c 
 * but has been rewritten to be easy to read and use, as no locks are now
 * needed anymore.
 *
 */

//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
//#include <linux/smp_lock.h>
#include <linux/usb.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include "../include/Adldev.h"
#include "../include/Adllib.h"
#include "../include/gpib_ioctl.h"
#include "gpib_user.h"
#include "gpib_proto.h"
#include "u348a.h"
#include "ezusbsys.h"

#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	#include <linux/smp_lock.h>    /* For (un)lock_kernel */
#endif


/* Define these values to match your devices */
#define USB_348A_VENDOR_ID	0x144A
#define USB_348A_ADLINK_ID	0x0B63
#define USB_348A_PRODUCT_ID	0x8052

#define KUSB_348A_VENDOR_ID	0x05E6
#define KUSB_348A_PRODUCT_ID	0x488B
/* table of devices that work with this driver */
static struct usb_device_id u348a_table [] = {
	{ USB_DEVICE(USB_348A_VENDOR_ID, USB_348A_PRODUCT_ID) },
	{ USB_DEVICE(USB_348A_ADLINK_ID, USB_348A_PRODUCT_ID) },
	{ USB_DEVICE(KUSB_348A_VENDOR_ID, KUSB_348A_PRODUCT_ID) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, u348a_table);


/* Get a minor range for your devices from the usb maintainer */
#define USB_348A_MINOR_BASE	192

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
  #define init_MUTEX(sem)         sema_init(sem, 1)
  #define DECLARE_MUTEX(name)     \
          struct semaphore name = __SEMAPHORE_INITIALIZER(name, 1)
#endif

#define to_u348a_dev(d) container_of(d, U348A_DEVEXT, kref)

static struct usb_driver u348a_driver;

MODULE_LICENSE("GPL");

//prototype for ioctl
extern int read_kioctl( gpib_file_private_t *file_priv, PU348A_DEVEXT board,
	unsigned long arg);
extern int write_kioctl( gpib_file_private_t *file_priv, PU348A_DEVEXT board,
	unsigned long arg);
extern int command_kioctl( gpib_file_private_t *file_priv, PU348A_DEVEXT board,
	unsigned long arg);
extern int open_dev_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int close_dev_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int serial_poll_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int wait_kioctl( gpib_file_private_t *file_priv, PU348A_DEVEXT board, unsigned long arg );
extern int parallel_poll_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int online_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int remote_enable_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int take_control_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int line_status_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int pad_kioctl( PU348A_DEVEXT board, gpib_file_private_t *file_priv,
	unsigned long arg );
extern int sad_kioctl( PU348A_DEVEXT board, gpib_file_private_t *file_priv,
	unsigned long arg );
extern int eos_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int request_service_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int autospoll_kioctl(PU348A_DEVEXT board, unsigned long arg);
extern int mutex_kioctl( PU348A_DEVEXT board, gpib_file_private_t *file_priv, unsigned long arg );
extern int timeout_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int status_bytes_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int sta_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int board_info_kioctl( const PU348A_DEVEXT board, unsigned long arg );
extern int ppc_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int query_board_rsv_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int interface_clear_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int event_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int request_system_control_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int t1_delay_kioctl( PU348A_DEVEXT board, unsigned long arg );
extern int ibgts( PU348A_DEVEXT board );
extern gpib_descriptor_t* get_descriptor( const gpib_file_private_t *file_priv,
	int handle );

static void u348a_delete(struct kref *kref)
{	
  U348A_DEVEXT *dev = to_u348a_dev(kref);

  mutex_destroy(&dev->io_mutex);

  if (dev->private_data )
   kfree( dev->private_data );
  dev->private_data = NULL;

  //kernel buffer free
  if(dev->buffer)
  	kfree( dev->buffer );
   usb_put_dev(dev->udev);
   kfree (dev);
}

uint8_t ioport_read_byte_with_lock(U348A_DEVEXT *priv, unsigned int register_num)
{
  uint8_t retval = 0;	
  int ret = 0;
  ret = UDServiceFire(priv, VR_GPIB_SREG, register_num & 0xffff, 0, 1, &retval, 1);	
  if (ret < 0) 
  {  
      dev_err(&priv->udev->dev, "U348A - %s - retval=%d\n",__FUNCTION__,ret); 
  }	
	
  return retval;
}

void ioport_write_byte_with_lock(U348A_DEVEXT *priv, unsigned char data, unsigned int register_num)
{
	//unsigned long flags;
  int ret = 0;
  VENDOR_REQUEST_IN vr;
  unsigned char setBuf[2];
	
  memset (&vr, 0, sizeof(VENDOR_REQUEST_IN));	
  setBuf[0] = data;
  ret = UDServiceFire(priv,VR_GPIB_SREG,register_num&0xffff,0,1,setBuf, 0);	
  if (ret < 0) 
  {  
    dev_err(&priv->udev->dev, "U348A - %s - retval=%d\n",  __FUNCTION__, ret ); 
  }		
}

int ioport_reads_with_lock(U348A_DEVEXT *priv, unsigned int register_num, U8 index, uint8_t len, uint8_t *rdbuf)
{
   //unsigned long flags;
   int ret = 0, retval=0;

   ret = UDServiceFire(priv, VR_GPIB_SREG, register_num & 0xff, index, len, rdbuf, 1);
  if (ret < 0) 
  {  
     dev_err(&priv->udev->dev, "U348A - %s - retval=%d\n",  __FUNCTION__, ret); 
      retval = -1;
  } else
      retval = 0;
  return retval;
}

int ioport_Mult_Read_write(U348A_DEVEXT *priv, uint8_t dir, uint8_t index, uint8_t length, uint8_t* buffer)
{
//	unsigned long flags;
	VENDOR_REQUEST_IN vr;
	int retval=0;
	
	memset (&vr, 0, sizeof(VENDOR_REQUEST_IN));	
	//KeAcquireSpinLock( &priv->register_page_lock, &kIrql);
	vr.bmRequestType = 0x40;
	vr.bRequest = VR_GPIB_REG;
	vr.wValueL = dir; //r:1, w:0
	vr.wIndexL = index; //0: read from gpib, 1 & 2: read from 8051 shadow
	vr.wLengthL = length;
	//KdPrint(("s0:\n"));
	retval = UDServiceFire(priv, VR_GPIB_REG, vr.wValueL, vr.wIndexL, vr.wLengthL, buffer, 0);
  if (retval < 0) 
  {  
      dev_err(&priv->udev->dev, "U348A - %s - retval=%d\n",  __FUNCTION__, retval ); 
      return retval;
  }	

  if(dir) {
    memset (&vr, 0, sizeof(VENDOR_REQUEST_IN));
    vr.bmRequestType = 0xC0;
    vr.bRequest = VR_GPIB_RESPONSE;
    vr.wLengthL = length+1;
    retval = UDServiceFire(priv, vr.bRequest, 0, 0, vr.wLengthL, buffer, 1);
    if (retval < 0) 
    {  
      dev_err(&priv->udev->dev, "U348A - %s - retval=%d\n",  __FUNCTION__, retval ); 
      return retval;
    }			
  }
  return retval; 
}

void u348a_board_reset( U348A_DEVEXT *priv )
{
   uint8_t rwbuf[128];
   uint8_t rwcnt = 0;

   if( test_bit( RFD_HOLDOFF_BN, &priv->state )) //&&
   {
     short line_status=0;

     line_status = iblines( priv );
     if(((!(line_status & BusATN)) && (line_status & BusNRFD))||(line_status & BusATN)) {
	test_and_clear_bit_sync( priv, RFD_HOLDOFF_BN, &priv->state );

        write_byte( priv, AUX_FH, AUXMR );
     }
   }
   memset(rwbuf, 0, 128);
   rwcnt = 0;
   rwbuf[rwcnt++] = AUXMR;
   rwbuf[rwcnt++] = AUX_CR;
	/* 7210 chip reset */
	//...write_byte(priv, AUX_CR, AUXMR);        
	/* disable all interrupts */
   priv->reg_bits[ IMR1 ] = 0;
   rwbuf[rwcnt++] = IMR1;
   rwbuf[rwcnt++] = 0;
	//...write_byte(priv, priv->reg_bits[ IMR1 ], IMR1);
   priv->reg_bits[ IMR2 ] = 0;
   rwbuf[rwcnt++] = IMR2;
   rwbuf[rwcnt++] = 0;
	//...write_byte(priv, priv->reg_bits[ IMR2 ], IMR2);
   rwbuf[rwcnt++] = SPMR;
   rwbuf[rwcnt++] = 0;
	//...write_byte(priv, 0, SPMR);
   ioport_Mult_Read_write(priv, 0, 0, rwcnt, rwbuf); 
	/* clear registers by reading */
   memset(rwbuf, 0, 128);
   rwcnt = 0;
   rwbuf[rwcnt++] = CPTR;
   rwbuf[rwcnt++] = ISR1;	
   rwbuf[rwcnt++] = ISR2;
   ioport_Mult_Read_write(priv, 1, 0, rwcnt, rwbuf); 	
	//...read_byte(priv, CPTR);
	//...read_byte(priv, ISR1);
	//...read_byte(priv, ISR2);
   memset(rwbuf, 0, 128);
   rwcnt = 0;
	/* parallel poll unconfigure */	
   rwbuf[rwcnt++] = AUXMR;
   rwbuf[rwcnt++] = PPR | HR_PPU;
	//....write_byte(priv, PPR | HR_PPU, AUXMR);
   priv->reg_bits[ ADMR ] = HR_TRM0 | HR_TRM1;	
   priv->auxa_bits = AUXRA | HR_HLDE;	
   rwbuf[rwcnt++] = AUXMR;
   rwbuf[rwcnt++] = priv->auxa_bits;
	//...write_byte(priv, priv->auxa_bits, AUXMR);
   rwbuf[rwcnt++] = AUXMR;
   rwbuf[rwcnt++] = AUXRE | 0;
	//...write_byte( priv, AUXRE | 0, AUXMR );
	/* set INT pin to active high, enable command pass through of unknown commands */
	priv->auxb_bits = AUXRB /*| HR_CPTE*/ | HR_INV;
	rwbuf[rwcnt++] = AUXMR;
	rwbuf[rwcnt++] = priv->auxb_bits;
	//...write_byte(priv, priv->auxb_bits, AUXMR);
	rwbuf[rwcnt++] = AUXMR;
	rwbuf[rwcnt++] = AUXRE;
	rwbuf[rwcnt++] = AUXMR;
	rwbuf[rwcnt++] = AUXRF;			
	//...write_byte(priv, AUXRE, AUXMR);	
        priv->auxg_bits = AUXRG | HR_NTNL |HR_CHES;// |HR_DISTCT;
       	rwbuf[rwcnt++] = AUXMR;
	rwbuf[rwcnt++] = priv->auxg_bits;	 
	//....write_byte(priv, priv->auxg_bits, AUXMR);		
	priv->auxi_bits = AUXRI | HR_SISB;// HR_USTD;
     	rwbuf[rwcnt++] = AUXMR;
	rwbuf[rwcnt++] = priv->auxi_bits;	
	//...write_byte(priv, priv->auxi_bits, AUXMR);			  	   	
	ioport_Mult_Read_write(priv, 0, 0, rwcnt, rwbuf);
	//set LON off
	////UDServiceFire(priv->pSelfFDO, VR_GPIB_LISTENER, 0, 0, 0, NULL, 0);	
	priv->state = 0;
	priv->status = 0;

 	priv->dmao = 0;
 	priv->dmai = 0;
	priv->spe = 0;
	priv->seoi=0;
	//////
	 priv->usec_timeout = 3000000;
   priv->parallel_poll_configuration = 0;
   priv->ppoll_usec_timeout = 2;
   /////priv->online = 0;
   priv->autospollers = 0;
   priv->autospoll_pid = 0;
   //yuan add 03/03/08
   priv->locking_pid = 0;
	//init_MUTEX_LOCKED(&board->autospoll_completion);
	//init_event_queue( &board->event_queue );
   priv->minor = -1;
	//init_gpib_pseudo_irq(&board->pseudo_irq);
   //yuan 05/26/06 modify to set board as a slave at startup
   priv->master = 0;
   //pDevExt->master = 1;
   priv->exclusive = 0;   
   priv->stuck_srq = 0;
}

void u348a_board_online( U348A_DEVEXT *priv)
{
	U8 rwbuf[128];
	U8 rwcnt = 0;
	//set pa
	adgpib_primary_address( priv, priv->pad );
	//set sa
	adgpib_secondary_address( priv, priv->sad, priv->sad >= 0 );
	memset(rwbuf, 0, 128);
	rwbuf[rwcnt++] = IMR1;
	rwbuf[rwcnt++] = priv->reg_bits[ IMR1 ];		
	rwbuf[rwcnt++] = IMR2;
	rwbuf[rwcnt++] = priv->reg_bits[ IMR2 ];
	rwbuf[rwcnt++] = AUXMR;
	rwbuf[rwcnt++] = AUX_PON;
	ioport_Mult_Read_write(priv, 0, 0, rwcnt, rwbuf);	
}

void init_gpib_descriptor( gpib_descriptor_t *desc )
{
	memset(desc, 0, sizeof(gpib_descriptor_t));
	desc->pad = 0;
	desc->sad = -1;
	desc->is_board = 0;
	desc->io_in_progress = 0;
}

static int init_gpib_file_private( gpib_file_private_t *priv )
{
	priv->holding_mutex = 0;
	memset( priv->descriptors, 0, sizeof( priv->descriptors ) );
	priv->descriptors[ 0 ] = kmalloc(sizeof( gpib_descriptor_t ), GFP_KERNEL );
	if( priv->descriptors[ 0 ] == NULL )
	{
		return -ENOMEM;
	}
	init_gpib_descriptor( priv->descriptors[ 0 ] );
	priv->descriptors[ 0 ]->is_board = 1;

	return 0;
}

static int free_open_devices( gpib_file_private_t *file_priv, PU348A_DEVEXT board )
{
	int retval = 0;
	int i;
	
	for( i = 0; i < GPIB_MAX_NUM_DESCRIPTORS; i++ )
	{
		gpib_descriptor_t *desc;

		desc = file_priv->descriptors[ i ];
		if( desc == NULL ) continue;
	
		if( desc->is_board == 0 )
		{
			retval = remove_open_device( &board->device_list, desc->pad,
				desc->sad );
			if( retval < 0 ) return retval;
		} 
		
		kfree( desc );
		file_priv->descriptors[ i ] = NULL;
	}

	return 0;
}

static int u348a_open(struct inode *inode, struct file *file)
{
	U348A_DEVEXT *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0, rwcnt=0;
	U8 rwbuf[128];

	subminor = iminor(inode);
	//printk ("%s before open device for minor %d\n", __FUNCTION__, subminor);
	interface = usb_find_interface(&u348a_driver, subminor);
	if (!interface) {
		printk ("%s - error, can't find device for minor %d\n",
		     __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit;
	}
	printk ("%s open device for minor %d\n", __FUNCTION__, subminor);
	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

   if ( dev->Reference ) 
   {
	dev->Reference ++;
	return 0;
   }
   u348a_board_reset( dev );
    dev->t1_nano_sec = adgpib_t1_delay( dev, 500 );
	/* set clock register for maximum (20 MHz) driving frequency
	 * ICR should be set to clock in megahertz (1-15) and to zero
	 * for clocks faster than 15 MHz (max 20MHz) */
    memset(rwbuf, 0, 128);
    rwbuf[rwcnt++] = AUXMR;
    rwbuf[rwcnt++] = ICR | 0;	    	 
    ioport_Mult_Read_write(dev, 0, 0, rwcnt, rwbuf);
    //...write_byte(pDevExt, ICR | 0, AUXMR);
    dev->dwDevRA = dev->dwDevRB = 0;
    //yuan 06/17/06 modify to set the device as a slave
    dev->hs_mode_bits = HS_TX_ENABLE|HS_RX_ENABLE;//|HS_SYS_CONTROL;
    u348a_board_online( dev );
    dev->online = 1;
    dev->private_data = kmalloc(sizeof( gpib_file_private_t ), GFP_KERNEL );
    if(!dev->private_data) {
		    return -ENOMEM;
    }
    if(init_gpib_file_private( ( gpib_file_private_t * ) dev->private_data ))
	     return -ENOMEM;
    dev->cur_taddr = UNT;
 		
	  /* increment our usage count for the device */
	  kref_get(&dev->kref);

	  /* save our object in the file's private structure */
	  file->private_data = dev;
    dev->Reference ++;
exit:
	return retval;
}

static int u348a_release(struct inode *inode, struct file *file)
{
   U348A_DEVEXT *dev;
   gpib_file_private_t *priv;
  
   dev = (U348A_DEVEXT *) (file->private_data);
   if (dev == NULL)
	return -ENODEV;
   if(dev->Reference>1)
   {
	dev->Reference --;
	return 0;	
   }

   priv = dev->private_data;
   if( priv ) {
	free_open_devices( priv, dev );
        kfree( dev->private_data );
	dev->private_data = NULL;
    }
    u348a_board_reset( dev );
    dev->hs_mode_bits = HS_TX_ENABLE|HS_RX_ENABLE;
    dev->exclusive = 0;
   
    dev->Reference --;

    /* decrement the count on our device */
    kref_put(&dev->kref, u348a_delete);
    return 0;
}

int u348a_ioctl( struct inode *inode,  struct file *file,
    unsigned int ioctl_num, unsigned long arg)
{
  int result=0;
  void* temp_param;
  uint8_t rwbuf[128];
 
  U348A_DEVEXT* pDevExt = (U348A_DEVEXT*)(file->private_data);
  memset(rwbuf, 0, 128);

  switch( ioctl_num )
  {
#if 0		
	   	case SW_IOC_GET_RESRC :
     		temp_param = (void *)kmalloc( sizeof( SW_IOT_RESRC ), GFP_ATOMIC );
      {
				struct usb_host_interface *iface_desc;
			struct usb_endpoint_descriptor *endpoint;
      	
        PUSBD_INTERFACE_INFORMATION interfaceInfo = NULL;
        PUSBD_PIPE_INFORMATION     pipeInfo = NULL;
        interfaceInfo = pDevExt->Interface;
		   	if (!interfaceInfo)
   			{
      			goto IoctlError;      	
   			}
      	pipeInfo = &(interfaceInfo->Pipes[3]);
				((SW_IOT_RESRC*)temp_param)->PortAddr[0] = (unsigned int) (pipeInfo->MaximumPacketSize);
				PSB_RESRC->PortAddr[1] = (unsigned int) (pipeInfo->MaximumTransferSize);
			  pipeInfo = &(interfaceInfo->Pipes[2]);
				PSB_RESRC->PortAddr[2] = (unsigned int) (pipeInfo->MaximumPacketSize);
				PSB_RESRC->PortAddr[3] = (unsigned int) (pipeInfo->MaximumTransferSize);	
		}
     		
		for(vi=0; vi<6; vi++)
		      ((SW_IOT_RESRC*)temp_param)->PortAddr[vi] = pDevExt->dwPortAddr[vi];
		     if(copy_to_user((void *)arg, (const void *)temp_param, sizeof(SW_IOT_RESRC))){
               		kfree(temp_param);
               		return -EFAULT;
     		}
		kfree( temp_param );
		return DAS_ERR_NO;
     		break;
#endif
	case GBIP_IOC_PORT_RD :
   	 temp_param = (void *)kmalloc( sizeof(read_write_ioctl_t), GFP_ATOMIC );
	 if ( !((read_write_ioctl_t*)temp_param)->count) {
	   kfree( temp_param );
	   return DAS_ERR_NO;
	 }
	 ioport_reads_with_lock (pDevExt, \
	 ((read_write_ioctl_t*)temp_param)->handle, \
 	 (uint8_t)((read_write_ioctl_t*)temp_param)->end,\
	 (uint8_t) ((read_write_ioctl_t*)temp_param)->count, rwbuf);
	 memcpy(((read_write_ioctl_t*)temp_param)->buffer, rwbuf, (uint8_t) ((read_write_ioctl_t*)temp_param)->count);
	 if(copy_to_user((void *)arg, (const void *)temp_param, sizeof(read_write_ioctl_t)))
         {
         	kfree(temp_param);
         	return -EFAULT;
         }
	 kfree( temp_param );
	 return DAS_ERR_NO;
	 break;
	case GBIP_IOC_PORT_WR :
   	 temp_param = (void *)kmalloc( sizeof(read_write_ioctl_t), GFP_ATOMIC );
	 if ( !((read_write_ioctl_t*)temp_param)->count) {
	   kfree( temp_param );
	   return DAS_ERR_NO;
	 }
	 memcpy(rwbuf, ((read_write_ioctl_t*)temp_param)->buffer, (U8) ((read_write_ioctl_t*)temp_param)->count);
	 ioport_Mult_Read_write(pDevExt, 0, (U8) ((read_write_ioctl_t*)temp_param)->end, (U8) ((read_write_ioctl_t*)temp_param)->count, rwbuf);
	 if(copy_to_user((void *)arg, (const void *)temp_param, sizeof(read_write_ioctl_t))){
               	kfree(temp_param);
               	return -EFAULT;
         }
	 kfree( temp_param );
	 return DAS_ERR_NO;
	 break;		

	case IBAUTOSPOLL:
		return autospoll_kioctl(pDevExt, arg);
		break;
	case IBBOARD_INFO:
		return board_info_kioctl( pDevExt, arg );
		break;
	case IBPAD:
		return pad_kioctl( pDevExt, pDevExt->private_data, arg );
		break;
	case IBSAD:
		return sad_kioctl( pDevExt, pDevExt->private_data, arg );
		break;
	case IBONL:
		return online_kioctl( pDevExt, arg );
		break;
	default:
		break;
	}

	if( !pDevExt->online )
	{
		printk( "board has to be on line for ioctl_num %d \n",
			ioctl_num & 0xff );
		return -EINVAL;
	}

	switch( ioctl_num )
	{
		case IBCLOSEDEV:
			return close_dev_kioctl( pDevExt, arg );
			break;
		case IBOPENDEV:
			return open_dev_kioctl( pDevExt, arg );
			break;
		case IBSPOLL_BYTES:
			return status_bytes_kioctl( pDevExt, arg );
			break;
		case IBWAIT:
			return wait_kioctl( pDevExt->private_data, pDevExt, arg );
			break;
		case IBSTA:
			return sta_kioctl( pDevExt, arg );			
			break;
		case IBLINES:
			return line_status_kioctl( pDevExt, arg );
			break;
		case IBLOC:
			write_byte( pDevExt, AUX_RTL, AUXMR);
			return 0;
			break;
		case IBQUERY_BOARD_RSV:
			return query_board_rsv_kioctl( pDevExt, arg );
			break;
		case SW_IOC_ASYNC_STOP:
			{
	        	  gpib_descriptor_t *desc;
			  temp_param = (void *)kmalloc( sizeof( pad_ioctl_t ), GFP_ATOMIC );
		          desc = get_descriptor( pDevExt->private_data, ((pad_ioctl_t*)temp_param)->handle );
			  if(desc && (desc->io_in_progress == 1))	
			  {		   
			      desc->abort = 1;
			      pDevExt->io_done = 1;	
			  }
			  kfree( temp_param );
			  return DAS_ERR_NO; 
			}
		default:
			break;
	}

	if( pDevExt->locking_pid && (current->pid != pDevExt->locking_pid) )
	{
		printk( "pid locking %i\n", ioctl_num & 0xff );
		return -EPERM;
	}

	switch( ioctl_num )
	{
		case IB_T1_DELAY:
			return t1_delay_kioctl( pDevExt, arg );
			break;
		case IBCAC:
			return take_control_kioctl( pDevExt, arg );
			break;
		case IBCMD:
			return command_kioctl( pDevExt->private_data, pDevExt, arg );
			break;
		case IBEOS:
			return eos_kioctl( pDevExt, arg );
			break;
		case IBGTS:
			return ibgts( pDevExt );
			break;
		case IBPPC:
			return ppc_kioctl( pDevExt, arg );
			break;
		case IBRD:
			result = read_kioctl( pDevExt->private_data, pDevExt, arg );
			if(pDevExt->locking_pid)
				pDevExt->locking_pid = 0;
			pDevExt->io_done = 0;
			return result;			
			break;
		case IBRPP:
			return parallel_poll_kioctl( pDevExt, arg );
			break;
		case IBRSC:
			return request_system_control_kioctl( pDevExt, arg );
			break;
		case IBRSP:
			return serial_poll_kioctl( pDevExt, arg );
			break;
		case IBRSV:
			return request_service_kioctl( pDevExt, arg );
			break;
		case IBSIC:
			return interface_clear_kioctl( pDevExt, arg );
			break;
		case IBSRE:
			return remote_enable_kioctl( pDevExt, arg );
			break;
		case IBTMO:
			return timeout_kioctl( pDevExt, arg );
			break;
		case IBWRT:
			result = write_kioctl( pDevExt->private_data, pDevExt, arg );
			if(pDevExt->locking_pid)
				pDevExt->locking_pid = 0;
			pDevExt->io_done = 0;
			return result;			
			break;
		default:
			return -ENOTTY;
			break;
	}

	return -ENOTTY;
}


long u348a_unlocked_ioctl( struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
int ret = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
U348A_DEVEXT *pDevExt = file->private_data;
#endif

#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	lock_kernel();
#else
	mutex_lock(&pDevExt->io_mutex);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
	ret = u348a_ioctl( file->f_dentry->d_inode, file, ioctl_num, ioctl_param);
#else
	ret = u348a_ioctl( file_inode(file), file, ioctl_num, ioctl_param);
#endif

#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	unlock_kernel();
#else
	mutex_unlock(&pDevExt->io_mutex);
#endif
return (long) ret;
}

#if 0
static ssize_t u348a_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	struct usb_348a *dev;
	int retval = 0;

	dev = (struct usb_348a *)file->private_data;
	
	/* do a blocking bulk read to get data from the device */
	retval = usb_bulk_msg(dev->udev,
			      usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
			      dev->bulk_in_buffer,
			      min(dev->bulk_in_size, count),
			      &count, HZ*10);

	/* if the read was successful, copy the data to userspace */
	if (!retval) {
		if (copy_to_user(buffer, dev->bulk_in_buffer, count))
			retval = -EFAULT;
		else
			retval = count;
	}

	return retval;
}

static void u348a_write_bulk_callback(struct urb *urb, struct pt_regs *regs)
{
	/* sync/async unlink faults aren't errors */
	if (urb->status && 
	    !(urb->status == -ENOENT || 
	      urb->status == -ECONNRESET ||
	      urb->status == -ESHUTDOWN)) {
		printk("%s - nonzero write bulk status received: %d\n",
		    __FUNCTION__, urb->status);
	}

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length, 
			urb->transfer_buffer, urb->transfer_dma);
}

static ssize_t u348a_write(struct file *file, const char __user *user_buffer, size_t count, loff_t *ppos)
{
	struct usb_348a *dev;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;

	dev = (struct usb_348a *)file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		goto exit;

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent(dev->udev, count, GFP_KERNEL, &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}
	if (copy_from_user(buf, user_buffer, count)) {
		retval = -EFAULT;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
			  buf, count, u348a_write_bulk_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		printk("%s - failed submitting write urb, error %d\n", __FUNCTION__, retval);
		goto error;
	}

	/* release our reference to this urb, the USB core will eventually free it entirely */
	usb_free_urb(urb);

exit:
	return count;

error:
	usb_free_coherent(dev->udev, count, buf, urb->transfer_dma);
	usb_free_urb(urb);
	kfree(buf);
	return retval;
}
#endif

static struct file_operations u348a_fops = {
	//.owner =	THIS_MODULE,
	#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
    .unlocked_ioctl = u348a_unlocked_ioctl,
#else
    ioctl:   u348a_ioctl,  /*ioctl */
#endif
	.open =		u348a_open,
	.release =	u348a_release,
};

/* 
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with devfs and the driver core
 */
static struct usb_class_driver u348a_class = {
	.name = "usb/u348a%d",
	.fops = &u348a_fops,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)   
	.mode = S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
#endif
	.minor_base = USB_348A_MINOR_BASE,
};

static int u348a_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	U348A_DEVEXT *dev = NULL;
	//struct usb_host_interface *iface_desc;
	//struct usb_endpoint_descriptor *endpoint;
	int retval = -ENOMEM;
//printk("u348a_probe\n");
	/* allocate memory for our device state and initialize it */
	dev = kmalloc(sizeof(U348A_DEVEXT), GFP_KERNEL);
	if (dev == NULL) {
		printk("Out of memory\n");
		goto error;
	}
	memset(dev, 0x00, sizeof (*dev));
	kref_init(&dev->kref);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
#if 0	
	/* use only the first bulk-in and bulk-out endpoints */
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;    
		if (!dev->bulk_in_endpointAddr &&
		    (endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {
			/* we found a bulk in endpoint */
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				printk("Could not allocate bulk_in_buffer\n");
				goto error;
			}
		}

		if (!dev->bulk_out_endpointAddr &&
		    !(endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {
			/* we found a bulk out endpoint */
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
		}
	}
	
	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		printk("Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}
#endif
	dev->openable = 1;
  init_waitqueue_head( &(dev->sync_wq) );
  dev->pad = 0;
  dev->sad = -1;
	dev->usec_timeout = 3000000;
	dev->parallel_poll_configuration = 0;
  dev->ppoll_usec_timeout = 2;
  dev->online = 0;
  dev->autospollers = 0;
  dev->autospoll_pid = 0;
  dev->minor = -1;
  dev->master = 0;
  dev->exclusive = 0;   
  dev->stuck_srq = 0;
  dev->dmao = 0;
  dev->dmai = 0;

  dev->buffer_length = 0x10000;
  dev->buffer = kmalloc(dev->buffer_length, GFP_KERNEL );
//spinlock init
	spin_lock_init(&(dev->spinlock));
	spin_lock_init(&(dev->register_page_lock));
//mutex init
	init_MUTEX(&dev->autopoll_mutex);
	init_MUTEX(&dev->spoll_mutex);
	mutex_init(&dev->io_mutex);
	init_MUTEX(&dev->req_mutex);
//init list
	INIT_LIST_HEAD( &dev->device_list );
//timer
//modify 2018/10/09
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
      init_timer(&dev->adtimer);
#else
      timer_setup(&dev->adtimer, timeout_routine, 0);
#endif

	
////


	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &u348a_class);
	if (retval) {
		/* something prevented us from registering this driver */
		printk("Not able to get a minor for this device.\n");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	printk("USB-3488 device now attached to u348a-%d\n", interface->minor);
	return 0;

error:
	if (dev)
		kref_put(&dev->kref, u348a_delete);
	return retval;
}

static void u348a_disconnect(struct usb_interface *interface)
{
	U348A_DEVEXT *dev;
	int minor = interface->minor;

	/* prevent skel_open() from racing skel_disconnect() */
//	lock_kernel();

  //mutex_lock( &u348a_mutex ); // not interruptible 
	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* give back our minor */
	usb_deregister_dev(interface, &u348a_class);
  //mutex_unlock( &u348a_mutex ); // not interruptible 
//	unlock_kernel();

	/* decrement our usage count */
	kref_put(&dev->kref, u348a_delete);

	printk("USB Skeleton #%d now disconnected\n", minor);
}

static struct usb_driver u348a_driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	.owner = THIS_MODULE,
#endif	
	.name = "u348a",
	.id_table = u348a_table,
	.probe = u348a_probe,
	.disconnect = u348a_disconnect,
};

static int __init usb_348a_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&u348a_driver);
	if (result)
		printk("usb_register failed. Error number %d\n", result);
printk("usb_register\n");
	return result;
}

static void __exit usb_348a_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&u348a_driver);
printk("usb_deregister\n");
}

module_init (usb_348a_init);
module_exit (usb_348a_exit);

MODULE_LICENSE("GPL");
