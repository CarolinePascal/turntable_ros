/***************************************************************************
 Adldev.h
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from include/gpib_types.h of the Linux GPIB Package driver 
              by Frank Mori Hess      
 Copyright (C) 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/
 
#ifndef		_ADLDEV_H_
#define		_ADLDEV_H_

#include <linux/pci.h> // struct pci_dev
#include <linux/timer.h>
#include <linux/version.h>

#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) ( ((a)<16) | ((b)<8) | (c) )
#endif

#include <linux/wait.h> // for DECLARE_WAIT_QUEUE_HEAD 
#include <linux/spinlock.h> // struct spinlock_t
#include <linux/semaphore.h> // struct semphore
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
// Constant for structures
// Port report from PCI bios
#define		PCI_PORT_MAX		6
#define		PCI_CTRL_P9030		3

/*------------------------------------------------------------------*/
// #pragma pack(push,1)

#ifndef basic_type
#define basic_type
typedef unsigned char   U8;
typedef short           I16;
typedef unsigned short  U16;
typedef int            I32;
typedef unsigned int   U32;
typedef long            I64;
typedef unsigned long   U64;

typedef enum { FALSE, TRUE } BOOLEAN;

#define FIRSTBYTE(VALUE)  (VALUE & 0x00FF)
#define SECONDBYTE(VALUE) ((VALUE >> 8) & 0x00FF)
#define THIRDBYTE(VALUE) ((VALUE >> 16) & 0x00FF)
#define FOURTHBYTE(VALUE) ((VALUE >> 24) & 0x00FF)

#endif // basic_type

#define  ADL_PCI_MEMORY_MAX		6
#define  ADL_PCI_PORT_MAX		6
#define  ADL_PCI_FUNC_COUNT     4

#define GPIB_MAX_NUM_DESCRIPTORS 0x1000
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
// PCI Address / Port Struct -- Physical Address, Base Address, Length
typedef struct
pci_buffer_adderss
{
	unsigned long		dwAddPhy;
	unsigned long		dwAddMap;
	unsigned long		dwLength;
}PCI_ADD, *PPCI_ADD;
/*------------------------------------------------------------------*/
typedef struct _FIFO_BUF
{
	uint8_t *buffer;
	signed short cur_pos;
	unsigned char r_end : 1;
} FIFO_BUF, *PFIFO_BUF;

typedef struct
{
	/* list_head so we can make a linked list of devices */
	struct list_head list;
	unsigned int pad;	/* primary gpib address */
	int sad;	/* secondary gpib address (negative means disabled) */
	/* stores serial poll bytes for this device */
	struct list_head status_bytes;
	unsigned int num_status_bytes;
	/* number of times this address is opened */
	unsigned int reference_count;
	unsigned int usec_timeout;
	PFIFO_BUF fifo_buf;
	/* flags loss of status byte error due to limit on size of queue */
	unsigned dropped_byte : 1;
} gpib_status_queue_t;

typedef struct
{
	struct list_head list;
	unsigned char poll_byte;
} status_byte_t;

/* Used to store device-descriptor-specific information */
typedef struct
{
	unsigned int pad;	/* primary gpib address */
	int sad;	/* secondary gpib address (negative means disabled) */
	volatile short io_in_progress;
	volatile short abort;
	unsigned int usec_timeout;
	PFIFO_BUF fifo_buf;
	unsigned is_board : 1;
} gpib_descriptor_t;

typedef struct
{
	volatile short holding_mutex;
	gpib_descriptor_t *descriptors[ GPIB_MAX_NUM_DESCRIPTORS ];
} gpib_file_private_t;

typedef struct
{
	struct list_head event_head;
	spinlock_t	 lock;
	unsigned int num_events;
	unsigned dropped_event : 1;
} gpib_event_queue_t;

/*------------------------------------------------------------------*/
// device extension for PCI/PCIe-348A
/*------------------------------------------------------------------*/
typedef struct pci_device_extension
{
	
	U8		  wBus;
  U16     wDevFunc;
	U32		  dwMemoryCount;
	U32		  dwPortCount;
	U32		  dwIrqCount;
	unsigned long  dwMemAddr[ADL_PCI_MEMORY_MAX];
	U32		  dwPortLen[PCI_PORT_MAX];
	U32		  dwPortAddr[ADL_PCI_PORT_MAX];
	volatile void __iomem *ioaddr[ADL_PCI_PORT_MAX];
	volatile void __iomem *iobarmap[ADL_PCI_PORT_MAX];
	U32		  dwMemLen[PCI_PORT_MAX];
	U16		  wCard;
  	U16     wDeviceID;
  	U16     wSubDeviceID;
	unsigned int	 Irq_no;
	spinlock_t	 spinlock;
	spinlock_t	 register_page_lock;
	volatile U8  reg_bits[ 8 ];
	volatile U8  page_reg_bits[ 8 ];
	volatile U8  auxa_bits;	// bits written to auxilliary register A
	volatile U8  auxb_bits;	// bits written to auxilliary register B
	volatile U8  auxf_bits;	// bits written to auxilliary register F
	volatile U8  auxg_bits;	// bits written to auxilliary register G
	volatile U8  auxi_bits;	// bits written to auxilliary register I	
	// used to keep track of board's state, bit definitions given below
	volatile unsigned long state;
	U8           type;
	uint8_t *buffer;
	/* length of buffer */
	unsigned int buffer_length;
	/* Used to hold the board's current status (see update_status() above)
	 */
	volatile unsigned long status;
	//wait_queue_head_t wait;
	volatile unsigned char hs_mode_bits;
	volatile unsigned out_fifo_half_empty : 1;
	volatile unsigned in_fifo_half_full : 1;
	void *private_data;
	struct list_head device_list;
	/* primary address */
	unsigned int pad;
	/* secondary address */
	int sad;
	/* timeout for io operations, in microseconds */
	unsigned int usec_timeout;
	/* board's parallel poll configuration byte */
	U8 parallel_poll_configuration;
	unsigned int ppoll_usec_timeout;
	/* t1 delay we are using */
	unsigned int t1_nano_sec;
	/* Count that keeps track of whether board is up and running or not */
	unsigned int online;
	/* number of processes trying to autopoll */
	int autospollers;
	/* pid of autospoll kernel thread */
	long autospoll_pid;
	struct semaphore autopoll_mutex;
	struct semaphore spoll_mutex;
	gpib_event_queue_t event_queue;
	volatile short io_in_progress;
	volatile short io_done;
	PFIFO_BUF fifo_buf;
	/* minor number for this board's device file */
	int minor;
	pid_t locking_pid; //yuan add 03/03/08
	/* Watchdog timer to enable timeouts */
	struct timer_list adtimer;
	/* struct to deal with polling mode*/
	//struct gpib_pseudo_irq pseudo_irq;
	/* Flag that indicates whether board is system controller of the bus */
	unsigned master : 1;
	/* Flag board has been opened for exclusive access */
	unsigned exclusive : 1;
	/* error dong autopoll */
	unsigned stuck_srq : 1;
	/* individual status bit */
	unsigned ist : 1;
	unsigned dmai : 1;
	unsigned dmao : 1;
	unsigned spe: 1; //yuan add 08/22/06
	unsigned seoi: 1; //yuan add 01/17/08	
	U32     cur_taddr;//eventRef[8];
  unsigned long     dwDevRA;
  unsigned long     dwDevRB;
	U16		  ThresHoldCnt;
	struct mutex      io_mutex; //add 2018/01/11
	U8 		  wRevId;
	U16		  wPortCnt;
	U16		  wMemCnt;
  //I16     MemPortIndex;
  void __iomem *dwMemMapAddr[ADL_PCI_MEMORY_MAX];
	BOOLEAN 	  openable;
	U16		  Reference;
	wait_queue_head_t sync_wq;
} PCI_DEVEXT, *PPCI_DEVEXT;

typedef struct pci_context_for_loading
{
	/*--- Fill By Caller ----*/
	U16			wVendorID;
	U16			wDeviceID;
//  jeffrey 00/04/14 , mark for linux driver
	U16			wPciController;
	U16			wPortCount;
	/*--Reserved ----*/
	PPCI_DEVEXT		pDevExt;
	U16			wCard;
	U16			wBus;
	U16			wDevice;
	U16			wFunc;
	U8			wRevId;
	//yuan add for testing status 01/07/00
}PCI_CONTEXT,*PPCI_CONTEXT;

/*------------------------------------------------------------------*/
//device extension for USB-348A
/*------------------------------------------------------------------*/
typedef struct USB348A_DEVEXT
{
	struct usb_device *	udev;			/* the usb device for this device */
	struct usb_interface *	interface;		/* the interface for this device */
	
	  /* 
     *  Buffer sizes 
     */  
    //size_t int_in_size;   
    size_t bulk_in_size;  
    size_t bulk_out_size;  
      
    /* 
     *  USB Endpoints 
     */  
    //__u8  int_in_endpointAddr;   
    __u8  bulk_in_endpointAddr;  
    __u8  bulk_out_endpointAddr;  
      
    /* 
     *  Endpoint intervals 
     */  
    //__u8  int_in_endpointInterval;   
    __u8  bulk_in_endpointInterval;  
    __u8  bulk_out_endpointInterval;  
      
    /* 
     *  URBs 
     */  
    struct urb * bulk_in_urb;  
    //struct urb * int_in_urb;   
    struct urb * bulk_out_urb;  
      
    /* 
     *  Refrence counter 
     */  
    struct kref      kref;  
  
    /* 
     *  Data from interrupt is retained here. 
     */  
    //struct switches_state  switches;   
  
    unsigned char notify;  
  
    /* 
     *  Track usage of the bulk pipes: serialize each pipe's use. 
     */  
    atomic_t bulk_write_available;  
    atomic_t bulk_read_available;  
  
    /* 
     *  Data tracking for Read/Write.  
     *  Writes will add to the pending_data count and  
     *  reads will deplete the pending_data count. 
     * 
     *  Note: The OSRFX2 device specs states that the firmware will buffer 
     *        up-to four write packet (two on EP6 and two on EP8). 
     *        The buffers can be drained by issuing reads to EP8. 
     *        The fifth outstanding write packet attempt will cause the write 
     *        to block, waiting for the arrival of a read request to 
     *        effectively free a buffer into which the write data can be  
     *        held. 
     */  
    size_t  pending_data;  	
	U16		  wCard;
  U16     wDeviceID;
  U16     wSubDeviceID;
	spinlock_t	 spinlock;
	spinlock_t	 register_page_lock;
	volatile U8  reg_bits[ 8 ];
	volatile U8  page_reg_bits[ 8 ];
	volatile U8  auxa_bits;	// bits written to auxilliary register A
	volatile U8  auxb_bits;	// bits written to auxilliary register B
	volatile U8  auxf_bits;	// bits written to auxilliary register F
	volatile U8  auxg_bits;	// bits written to auxilliary register G
	volatile U8  auxi_bits;	// bits written to auxilliary register I	
	// used to keep track of board's state, bit definitions given below
	volatile unsigned long state;
	U8           type;
	uint8_t *buffer;
	/* length of buffer */
	unsigned int buffer_length;
	/* Used to hold the board's current status (see update_status() above)
	 */
	volatile unsigned long status;
	//wait_queue_head_t wait;
	volatile unsigned char hs_mode_bits;
	volatile unsigned out_fifo_half_empty : 1;
	volatile unsigned in_fifo_half_full : 1;
	void *private_data;
	struct list_head device_list;
	/* primary address */
	unsigned int pad;
	/* secondary address */
	int sad;
	/* timeout for io operations, in microseconds */
	unsigned int usec_timeout;
	/* board's parallel poll configuration byte */
	U8 parallel_poll_configuration;
	unsigned int ppoll_usec_timeout;
	/* t1 delay we are using */
	unsigned int t1_nano_sec;
	/* Count that keeps track of whether board is up and running or not */
	unsigned int online;
	/* number of processes trying to autopoll */
	int autospollers;
	/* pid of autospoll kernel thread */
	long autospoll_pid;
	struct semaphore autopoll_mutex;
	struct semaphore spoll_mutex;
	struct semaphore req_mutex;
	gpib_event_queue_t event_queue;
	volatile short io_in_progress;
	volatile short io_done;
	PFIFO_BUF fifo_buf;
	/* minor number for this board's device file */
	int minor;
	pid_t locking_pid; //yuan add 03/03/08
	/* Watchdog timer to enable timeouts */
	struct timer_list adtimer;
	/* struct to deal with polling mode*/
	//struct gpib_pseudo_irq pseudo_irq;
	/* Flag that indicates whether board is system controller of the bus */
	unsigned master : 1;
	/* Flag board has been opened for exclusive access */
	unsigned exclusive : 1;
	/* error dong autopoll */
	unsigned stuck_srq : 1;
	/* individual status bit */
	unsigned ist : 1;
	unsigned dmai : 1;
	unsigned dmao : 1;
	unsigned spe: 1; 
	unsigned seoi: 1; 
	unsigned int      cur_taddr;
	unsigned long     dwDevRA;
	unsigned long     dwDevRB;
	U16		  ThresHoldCnt;
	struct mutex      io_mutex; 
	U8 		  wRevId;
	BOOLEAN 	  openable;
	U16		  Reference;
	wait_queue_head_t sync_wq;
} U348A_DEVEXT, *PU348A_DEVEXT;
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#endif


