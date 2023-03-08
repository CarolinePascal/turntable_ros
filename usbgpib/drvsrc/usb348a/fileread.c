#include <linux/version.h>
#include <linux/module.h>   /* Specifically, a module */
#include <linux/kernel.h>   /* We're doing kernel work */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) )
#include <linux/config.h>   // for PCI, SMP configuration
#endif
#include <asm/current.h> // current
#include <linux/mm.h>
#include <linux/fs.h> // locks_verify_area() , getname() , putname(), get_unused_fd() , put_unused_fd, filp_open()
#include <linux/dnotify.h>
#include <linux/fcntl.h>
#include <linux/file.h> // fget() , fput() , fd_install()
#include <asm/uaccess.h>  /* for put_user */
#include <asm/io.h>
#include <asm/delay.h>


#include "../include/Adldev.h"
#include "../include/Adllib.h"

//support with no Big Kernel Lock and linux 2.6.38
#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
        #include <linux/smp_lock.h>    /* For (un)lock_kernel */
#endif

#define SUCCESS 0

#define TMP_BUF_SIZE 80

static DEFINE_MUTEX(adgp_mutex);

#define getBit(reg_var, bit)   ((reg_var >> bit) & 1L)
#define setBit(reg_var, bit)   { reg_var = (reg_var | (1L << bit));  }
#define resetBit(reg_var, bit) { reg_var = (reg_var & ~(1L << bit)); }

/* Device Declarations **************************** */
asmlinkage ssize_t ker_open_file(const char *name, struct file ** pfile );
asmlinkage ssize_t ker_read_file( struct file *file, char * buf, size_t count );
asmlinkage void ker_close_file( struct file *file );

ssize_t FPGA_Download(PPCI_DEVEXT pDevExt);
char temp_buf[TMP_BUF_SIZE];

// modify from /usr/src/linux/kernel/acct.c
asmlinkage ssize_t ker_open_file(const char *name, struct file ** pfile )
{
	ssize_t error = -EPERM;
        struct file *file = NULL; 
//modify 2012/08/17
#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	lock_kernel();
#else
	mutex_lock(&adgp_mutex);
#endif	

if (name) {
		file = filp_open(name, O_RDONLY, 0);
		if (IS_ERR(file)) {
			error = PTR_ERR(file);
			goto out;
		}
		error = -EACCES;
//modify 2016/02/26
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
		if (!S_ISREG(file->f_dentry->d_inode->i_mode))
			goto out_err;
#else
		if (!S_ISREG(file_inode(file)->i_mode))
			goto out_err;
#endif

}

error = 0;
*pfile = file;
out:
	//unlock_kernel();
#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
	unlock_kernel();
#else
	mutex_unlock(&adgp_mutex);
#endif
	return error;
out_err:
	goto out;
}

#if 0
asmlinkage ssize_t ker_read_file(  struct file *file, char * buf, size_t count)
{
	mm_segment_t fs;
	struct inode *inode;
	ssize_t error = 0;

	fs = get_fs();
	set_fs(KERNEL_DS);
	inode = file->f_dentry->d_inode;
//	down(&inode->i_sem);
mutex_lock(&inode->i_mutex);
	error = file->f_op->read(file, buf, count, &file->f_pos);

	//up(&inode->i_sem);
mutex_unlock(&inode->i_mutex);
	set_fs(fs);

        return error;
}
#endif

asmlinkage void ker_close_file( struct file *file )
{
#if defined CONFIG_KERNEL_LOCK || LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
        lock_kernel();
        filp_close(file, NULL);
        unlock_kernel();
#else
        mutex_lock(&adgp_mutex);
        filp_close(file, NULL);
        mutex_unlock(&adgp_mutex);     
#endif
}

ssize_t FPGA_Download(PPCI_DEVEXT pDevExt)
{
#define  ADDR_FPGA_DATA   0x18
#define  ADDR_FPGA_CONF   0x1A
// CPLD: FPGA_STS @ 0x80
#define BIT_nConfig     0
#define BIT_CONF_DONE   2
#define BIT_BUSY        3
#define BIT_INIT_DONE   4
#define BIT_nStatus     6

  int  end_of_file, download_done, idx;
  U32  byte_cnt=0;
  int  temp_length=0, foffset=0;
  ssize_t error;
  char fpga_filename[40];
  struct file *file_4_read;
  U32 dwPort = PT_Port(pDevExt);
  unsigned char fpga_status = 0;

  strcpy( fpga_filename, "/etc/adgpib/fw/pci-gpib.rbf" );
  printk("file: %s\n", fpga_filename);
  if( (error = ker_open_file( fpga_filename, &file_4_read) )!= SUCCESS ){
      printk(" Kernel Open File failed !!!\n" );
	return FALSE;
  }
  //Toggle nConfig to Low
  outB( dwPort+ADDR_FPGA_CONF, 0x00 );
  //WRITE_PORT_UCHAR(ADDR_FPGA_CONF+dwPort, 0x00); 
  //mdelay(1);
   udelay(1000);
  do {
		fpga_status = inpB(ADDR_FPGA_CONF+dwPort);
	  //printf("Inital HandShake Before: nStatus :%X\n", fpga_sts);
  } while(getBit(fpga_status ,BIT_nStatus) == 1);  

  //Toggle nConfig to High
  outB(ADDR_FPGA_CONF+dwPort, 0x01); 
  do {
		fpga_status = inpB(ADDR_FPGA_CONF+dwPort);	    
  }while(getBit(fpga_status,BIT_nStatus) == 0);  
  
  printk("Inital Handshake DONE.\n");

  end_of_file = 0;
  //do {
  while (!end_of_file) {  
      // Read and output one byte from RBF file
      temp_length = kernel_read(file_4_read, foffset, temp_buf, TMP_BUF_SIZE);
      if( temp_length < 0 ) 
	    {
	     	ker_close_file( file_4_read );
	    	return temp_length; // some error occur !!
      }
      else if( temp_length < TMP_BUF_SIZE )
	      end_of_file = 1; // end of filev
      for (idx=0; idx < temp_length; idx++){
        outB(dwPort+ADDR_FPGA_DATA, temp_buf[idx]);
        byte_cnt++;
        do {
		      fpga_status = inpB(ADDR_FPGA_CONF+dwPort);
	      } while  (getBit(fpga_status, BIT_BUSY) == 1); 
      }
      //add 2016/02/26
      foffset += temp_length;      
  } //while (!end_of_file);
  ker_close_file( file_4_read );
  printk("FPGA Download DONE.\n");
  { int i; for (i=0; i<100; i++) udelay(10000); }
//  mdelay(1000); // Do not remove this line
  fpga_status = inpB(dwPort+ADDR_FPGA_CONF);
  //printk("%x\n", fpga_status);
  if (getBit(fpga_status, BIT_CONF_DONE) != 1) {
	  printk("Configuration Failed\n");
	  download_done = 0;
	  return FALSE;
  } else if (getBit(fpga_status, BIT_INIT_DONE) != 1) { 
	  printk("Initialization Failed!");	
	  download_done = 0;
	  return TRUE;
  } else {
    outB(dwPort+ADDR_FPGA_CONF, 0x03);
    udelay(10);
	  outB(dwPort+ADDR_FPGA_CONF, 0x01);
	  printk("download_done\n");
	  download_done = 1;
	  return TRUE;
  }
}
