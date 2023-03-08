//#include <linux/config.h>
#include <asm/current.h> // current
#include <linux/mm.h>
#include <linux/fs.h> // locks_verify_area() , getname() , putname(), get_unused_fd() , put_unused_fd, filp_open()
// #include <linux/smp_lock.h> // lock_kernel() , unlock_kernel()
#include <linux/fcntl.h>
#include <linux/file.h> // fget() , fput() , fd_install()
#include <linux/slab.h> 
#include <linux/uaccess.h>  /* for put_user */
#include <asm/delay.h>
#include <linux/usb.h>
#include <linux/version.h>
#include "usbgpib.h"

#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	#include <linux/smp_lock.h>    /* For (un)lock_kernel */
#endif

#define SUCCESS 0

#define TMP_BUF_SIZE 80
#define MAX_EP0_XFER_SIZE (1024*4) // 4K Control EP0 transfer limit imposed by OS
#define MAX_BULK_XFER_SIZE (1024*256)

#define DIRECTION_OUT 	0 
#define DIRECTION_IN  	1 

static DEFINE_MUTEX( fwdl_mutex );
/*****************************************************************************/  
/* This is the private device context structure.                             */  
/*****************************************************************************/  
#define VR_FPGA             0xA4 // configure FPGA

#define DIRECTION_OUT 	0 
#define DIRECTION_IN  	1 


/* Device Declarations **************************** */

asmlinkage ssize_t ker_open_file(const char *name, struct file ** pfile );
asmlinkage void ker_close_file( struct file *file );

ssize_t fx2_fpga_download(struct usb_device *	udev, char *fileName);

/* Send a vendor command to device */  
int vendor_command(struct usb_device *dev, unsigned char request,   
              unsigned short value, unsigned short index,  
              void *buf, int size, unsigned char In_direction )  
{  
int res = 0;

    if( In_direction == DIRECTION_IN )
    {
    	res = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),  
                   request,   
                   USB_DIR_IN | USB_TYPE_VENDOR,  
                   value,   
                   index, 
                   buf, 
                   size,  
                   USB_CTRL_GET_TIMEOUT);  
//printk("usb_control_msg_in: %d\n", res);
    }
    else
    {
    	res = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),  
                   request,   
                   USB_DIR_OUT | USB_TYPE_VENDOR,  
                   value,   
                   index, 
                   buf, 
                   size,  
                   USB_CTRL_SET_TIMEOUT);  
//printk("usb_control_msg_out: %d\n", res);
    }
return res;
}

// 2013-11-08, this might be swapped out, 
// move to fx2_fpga_download() /w kmalloc() 
//unsigned char temp_buf[MAX_EP0_XFER_SIZE];

// modify from /usr/src/linux/kernel/acct.c
// modify from /usr/src/linux/kernel/acct.c
asmlinkage ssize_t ker_open_file(const char *name, struct file ** pfile )
{
	ssize_t error = -EPERM;
        struct file *file = NULL; 
#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	lock_kernel();
#else
	mutex_lock(&fwdl_mutex);
#endif
	if (name) {
		file = filp_open(name, O_RDONLY, 0);
		if (IS_ERR(file)) {
			error = PTR_ERR(file);
			goto out;
		}
		error = -EACCES;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
		if (!S_ISREG(file->f_dentry->d_inode->i_mode))
			goto out_err;
#else
		if (!S_ISREG(file_inode(file)->i_mode))
			goto out_err;
#endif
//remove 2016/02/23
/**
		if (!S_ISREG(file->f_dentry->d_inode->i_mode))
			goto out_err;

		error = -EIO;
		if (!file->f_op->read)
			goto out_err;
**/
	}

	error = 0;
        *pfile = file;
out:
#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	unlock_kernel();
#else
	mutex_unlock(&fwdl_mutex);
#endif
	
	return error;
out_err:
	goto out;
}

asmlinkage void ker_close_file( struct file *file )
{
#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	lock_kernel();
	filp_close(file, NULL);
	unlock_kernel();
#else
	mutex_lock(&fwdl_mutex);
	filp_close(file, NULL);
	mutex_unlock(&fwdl_mutex);	
#endif
}


ssize_t fx2_fpga_download(struct usb_device *	udev, char *fileName)
{
    int  end_of_file, download_done, download_err; //, idx;
    int  temp_length=0;//, foffset=0;
    loff_t foffset = 0;
    ssize_t error;
    int retval;
    struct file *file_4_read;

    unsigned char* temp_buf = NULL;

    temp_buf = (unsigned char*)kmalloc( MAX_EP0_XFER_SIZE, GFP_KERNEL );
    if( temp_buf == NULL )
        return -ENOMEM;

    if( (error = ker_open_file( fileName, &file_4_read ) )!= SUCCESS ){
        printk(" [FPGA] file open failed in kernel mode \n" );
        return error;
    }
    //UDServiceFire(fdo, VR_FPGA, 0, 0, 0, NULL, 0);
    retval = vendor_command(udev, VR_FPGA, 0, 0, NULL, 0, DIRECTION_OUT);
    download_done = 0;
        
    //set file position to start of the file
    //rewind(rfp);
    //start to read fpga file and write to fpga
    end_of_file = 0;
    download_err = 0;
printk(" [FPGA] file %s open kernel mode \n",  fileName);
    while ( (end_of_file == 0) && (download_done == 0) )
    {
		//modify 2018/10/09
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
      temp_length = kernel_read(file_4_read, temp_buf, MAX_EP0_XFER_SIZE, &foffset);
#else
      temp_length = kernel_read(file_4_read, foffset, temp_buf, MAX_EP0_XFER_SIZE);
#endif
		
    //  temp_length = kernel_read(file_4_read, foffset, temp_buf, MAX_EP0_XFER_SIZE);
//printk(" [FPGA] read %d \n",  temp_length);
        if( temp_length < 0 ) 
        {
            ker_close_file( file_4_read );
			//modify 2018/10/09            
            printk(" [FPGA] ker_read() failed, error code - %d \n", temp_length );
			return 0;            
            //return temp_length; // some error occur !!
        }
        else if( temp_length < MAX_EP0_XFER_SIZE/*TMP_BUF_SIZE*/ )
            end_of_file = 1; // end of file

	//UDServiceFire(fdo, VR_FPGA, 1, end_of_file, bufferLength, ptr, 0);
        retval = vendor_command(udev, VR_FPGA, 1, end_of_file, temp_buf, temp_length, DIRECTION_OUT);
        if (retval < 0) 
        {  
            download_err = 1;
            printk(" [FPGA] vendor_command() failed, error code - %d \n", retval );
        }
        else
        {
     	   if( end_of_file == 1 )
               download_done = 1;
        }

		//modify 2018/10/09
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
		foffset += temp_length;
#endif		
        if (download_err == 1)
           break;

    }//while
        
    ker_close_file( file_4_read );
     
    if( temp_buf != NULL )
       kfree( temp_buf );

    if ( download_done == 0 )
    {
        printk(" [FPGA] FPGA reading failed !!\n" );
        return 0;
    }
printk(" [FPGA] download %d \n",  download_done);
  return 1;
}

int Ezusb_8051Reset(
   struct usb_device *	udev,
   unsigned char resetBit
   )
/*++

Routine Description:
   Uses the ANCHOR LOAD vendor specific command to either set or release the
   8051 reset bit in the EZ-USB chip.

Arguments:
   fdo - pointer to the device object for this instance of an Ezusb Device
   resetBit - 1 sets the 8051 reset bit (holds the 8051 in reset)
              0 clears the 8051 reset bit (8051 starts running)
              
Return Value:
   STATUS_SUCCESS if successful,
   STATUS_UNSUCCESSFUL otherwise

--*/
{
int retval = 0;
int8_t *rbit;
//printk(" [FPGA] Ezusb_8051Reset \n");
rbit= (unsigned char*)kmalloc( 1, GFP_KERNEL );
*rbit = resetBit;
    retval = vendor_command(udev, ANCHOR_LOAD_INTERNAL, CPUCS_REG_FX2, 0, rbit, 1, DIRECTION_OUT);
kfree(rbit);
    //UDServiceFire(fdo, ANCHOR_LOAD_INTERNAL, CPUCS_REG_FX2, 0, 1, &resetBit, 0);
	return retval;
}

ssize_t Firmware_Download_Int(
	struct usb_device *	udev,
	unsigned char * fImage, 
	size_t FileLen 
	)
{
   size_t   bufferLength;
   unsigned char *ptr,*bufptr ;//= fImage;
   int      end_of_file;
   size_t   rcnt;
//printk(" [FPGA] Firmware_Download_Int %d\n", FileLen);

    bufptr = (unsigned char*)kmalloc( FileLen, GFP_KERNEL );
    if( bufptr == NULL )
        return -ENOMEM;

   memcpy(bufptr, fImage, FileLen);
   ptr = bufptr;
   Ezusb_8051Reset(udev,1);
   end_of_file = 0;
   rcnt = 0;
   while (!end_of_file) {
	  if(FileLen - rcnt <= MAX_EP0_XFER_SIZE) {
            bufferLength = FileLen - rcnt;
            end_of_file = 1;
	  }
	  else
	   bufferLength = MAX_EP0_XFER_SIZE;

    //UDServiceFire(fdo, ANCHOR_LOAD_INTERNAL, rcnt, 0, bufferLength, ptr, 0);
    vendor_command(udev, ANCHOR_LOAD_INTERNAL, rcnt, 0, ptr, bufferLength, DIRECTION_OUT);
    rcnt += bufferLength;
    if(!end_of_file)
            ptr += bufferLength;
   }
   Ezusb_8051Reset(udev,0);
if( bufptr != NULL )
       kfree( bufptr );
  printk(" [FPGA] Firmware_Download_Int end \n");
   return 1;
}

ssize_t Firmware_Download_Ext(
	struct usb_device *	udev,
	unsigned char * fImage, 
	size_t FileLen 
	)
{
   size_t   bufferLength;
   unsigned char *ptr, *bufptr ;//= fImage;
   int      end_of_file;
   size_t   rcnt;
//printk(" [FPGA] Firmware_Download_Ext %d\n", FileLen);
    bufptr = (unsigned char*)kmalloc( FileLen, GFP_KERNEL );
    if( bufptr == NULL )
        return -ENOMEM;    
   memcpy(bufptr, fImage, FileLen);
   ptr = bufptr;
   end_of_file = 0;
   rcnt = 0;
   while (!end_of_file) {
	  if(FileLen - rcnt <= MAX_EP0_XFER_SIZE) {
            bufferLength = FileLen - rcnt;
            end_of_file = 1;
	  }
	  else 
	   bufferLength = MAX_EP0_XFER_SIZE;

    //UDServiceFire(fdo, VR_RAM, rcnt+0x4000, 0, bufferLength, ptr, 0);
    vendor_command(udev, VR_RAM, rcnt+0x4000, 0, ptr, bufferLength, DIRECTION_OUT);
    rcnt += bufferLength;
    if(!end_of_file)
            ptr += bufferLength;
   }
if( bufptr != NULL )
       kfree( bufptr );
  printk(" [FPGA] Firmware_Download_Ext done\n");
    return 1;
}
