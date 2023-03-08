/***************************************************************************
 gpib_ioctl.h
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from include/gpib_ioctl.h of the Linux GPIB Package driver 
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
 
#ifndef _GPIB_IOCTL_H
#define _GPIB_IOCTL_H

#include <asm/ioctl.h>

#define MAGIC_NUM 'P'
#define GPIB_MAX_NUM_BOARDS 16


static const int device_status_mask = ERR | TIMO | END | CMPL | RQS;
static const int board_status_mask = ERR | TIMO | END | CMPL | LOK | REM | CIC | ATN | TACS | LACS | DTAS | DCAS | SRQI;

//GPIB command message

enum cmd_byte
{
	GTL = 0x1,	
	SDC = 0x4,	
	PPConfig = 0x5,
#ifndef PPC
	PPC = PPConfig,	
#endif
	GET = 0x8,	
	TCT = 0x9,	
	LLO = 0x11,	
	DCL = 0x14,	
	PPU = 0x15,	
	SPE = 0x18,	
	SPD = 0x19,	
	LAD = 0x20,	
	UNL = 0x3F,	
	TAD = 0x40,	
	UNT = 0x5F,	
	SAD = 0x60,	
	PPE = 0x60,	
	PPD = 0x70	
};

//ppc/ppe
enum ppe_bits
{
	PPC_DISABLE = 0x10,
	PPC_SENSE = 0x8,
	PPC_DIO_MASK = 0x7
};

static __inline uint8_t MLA( unsigned int addr )
{
	return addr | LAD;
}

static __inline uint8_t MTA( unsigned int addr )
{
	return addr | TAD;
}

static __inline uint8_t MSA( unsigned int addr )
{
	return addr | SAD;
}

static __inline uint8_t PPE_byte( unsigned int dio_line, int sense )
{
	uint8_t cmd;

	cmd = PPE;
	if( sense )
		cmd |= PPC_SENSE;
	cmd |= ( dio_line - 1 ) & 0x7;
	return cmd;
}

static __inline int gpib_address_equal( unsigned int pad1, int sad1, unsigned int pad2, int sad2 )
{
	if( pad1 == pad2 )
	{
		if( sad1 == sad2 ) return 1;
		if( sad1 < 0 && sad2 < 0 ) return 1;
	}

	return 0;
}

static const int request_service_bit = 0x40;

enum gpib_events
{
	EventNone = 0,
	EventDevTrg = 1,
	EventDevClr = 2,
	EventIFC = 3
};
/*------------------------------------------------------------------*/
//#pragma pack(push,1)

typedef struct
{
	char name[100];
} board_type_ioctl_t;

typedef struct
{
	int handle;
	uint8_t *buffer;
	unsigned int count;
	int end;
	unsigned int usec_timeout;
} read_write_ioctl_t;

typedef struct
{
	unsigned int handle;
	unsigned int pad;
	int sad;
	unsigned int usec_timeout;
	unsigned is_board : 1;
} open_dev_ioctl_t;

typedef struct
{
	unsigned int handle;
} close_dev_ioctl_t;

typedef struct
{
	unsigned int pad;
	int sad;
	uint8_t status_byte;
} serial_poll_ioctl_t;

typedef struct
{
	int eos;
	int eos_flags;
} eos_ioctl_t;

typedef struct
{
	int handle;
	int wait_mask;
	int clear_mask;
	int set_mask;
	int ibsta;
	int pad;
	int sad;
	unsigned int usec_timeout;
} wait_ioctl_t;

typedef struct
{
	int online;
} online_ioctl_t;

typedef struct
{
	unsigned int num_bytes;
	unsigned int pad;
	int sad;
} spoll_bytes_ioctl_t;

typedef struct
{
	unsigned int pad;
	int sad;
	int parallel_poll_configuration;
	int autopolling;
	int is_system_controller;
	unsigned int t1_delay;
	unsigned ist : 1;
} board_info_ioctl_t;

typedef struct
{
	int pci_bus;
	int pci_slot;
} select_pci_ioctl_t;

typedef struct
{
	uint8_t config;
	unsigned set_ist : 1;
	unsigned clear_ist : 1;
}	ppoll_config_ioctl_t;

typedef struct
{
	unsigned char poll_byte;
	unsigned int ppoll_usec_timeout;
} ppoll_ioctl_t;

typedef struct
{
	unsigned int handle;
	unsigned int pad;
} pad_ioctl_t;

typedef struct
{
	unsigned int handle;
	int sad;
} sad_ioctl_t;

typedef struct
        _SW_IOT_RESRC
        {
        short           wErrorCode;
	  unsigned int   PortAddr[6];
        }
        SW_IOT_RESRC;
/*------------------------------------------------------------------*/
//#pragma pack(pop)

typedef short event_ioctl_t;
typedef int rsc_ioctl_t;
typedef unsigned int t1_delay_ioctl_t;
typedef short autospoll_ioctl_t;

//ioctl code
enum gpib_ioctl
{
	IBRD = _IOWR( MAGIC_NUM, 0, read_write_ioctl_t ),
	IBWRT = _IOWR( MAGIC_NUM, 1, read_write_ioctl_t ),
	IBCMD = _IOWR( MAGIC_NUM, 2, read_write_ioctl_t ),
	IBOPENDEV = _IOWR( MAGIC_NUM, 3, open_dev_ioctl_t ),			
	IBCLOSEDEV = _IOW( MAGIC_NUM, 4, close_dev_ioctl_t ),
	IBWAIT = _IOWR( MAGIC_NUM, 5, wait_ioctl_t ),
	IBRPP = _IOWR( MAGIC_NUM, 6, ppoll_ioctl_t ),
	
	IBSIC = _IOW( MAGIC_NUM, 9, unsigned int ),
	IBSRE = _IOW( MAGIC_NUM, 10, int ),
	IBGTS = _IO( MAGIC_NUM, 11 ),
	IBCAC = _IOW( MAGIC_NUM, 12, int ),
	IBLINES = _IOR( MAGIC_NUM, 14, short ),
	IBPAD = _IOW( MAGIC_NUM, 15, pad_ioctl_t ),
	IBSAD = _IOW( MAGIC_NUM, 16, sad_ioctl_t ),
	IBTMO = _IOW( MAGIC_NUM, 17, unsigned int ),
	IBRSP = _IOWR( MAGIC_NUM, 18, serial_poll_ioctl_t ),
	IBEOS = _IOW( MAGIC_NUM, 19, eos_ioctl_t ),
	IBRSV = _IOW( MAGIC_NUM, 20, uint8_t ),
	CFCBASE = _IOW( MAGIC_NUM, 21, unsigned long ),
	CFCIRQ = _IOW( MAGIC_NUM, 22, unsigned int ),
	CFCDMA = _IOW( MAGIC_NUM, 23, unsigned int ),
	CFCBOARDTYPE = _IOW( MAGIC_NUM, 24, board_type_ioctl_t ),

	IBMUTEX = _IOW( MAGIC_NUM, 26, int ),
	IBSPOLL_BYTES = _IOWR( MAGIC_NUM, 27, spoll_bytes_ioctl_t ),
	IBPPC = _IOW( MAGIC_NUM, 28, ppoll_config_ioctl_t ),
	IBBOARD_INFO = _IOR( MAGIC_NUM, 29, board_info_ioctl_t ),

	IBQUERY_BOARD_RSV = _IOR( MAGIC_NUM, 31, int ),
	IBSELECT_PCI = _IOWR( MAGIC_NUM, 32, select_pci_ioctl_t ),
	IBEVENT = _IOR( MAGIC_NUM, 33, event_ioctl_t ),
	IBRSC = _IOW( MAGIC_NUM, 34, rsc_ioctl_t ),
	IB_T1_DELAY = _IOW( MAGIC_NUM, 35, t1_delay_ioctl_t ),
	IBLOC = _IO( MAGIC_NUM, 36 ),
	IBONL = _IOW( MAGIC_NUM, 37, online_ioctl_t ),
	IBAUTOSPOLL = _IOW( MAGIC_NUM, 38, autospoll_ioctl_t ),
	SW_IOC_GET_RESRC = _IOWR( MAGIC_NUM, 39, SW_IOT_RESRC ),
        SW_IOC_ASYNC_STOP = _IOW( MAGIC_NUM, 40, pad_ioctl_t ),
	IBSTA = _IOWR( MAGIC_NUM, 41, wait_ioctl_t ),
      //SW_IOC_OPEN_EVENT = CTL_CODE( MAGIC_NUM, 42, METHOD_BUFFERED, FILE_ANY_ACCESS ),
	//GBIP_IOC_EEPROM_RW = CTL_CODE( MAGIC_NUM, 43, METHOD_BUFFERED, FILE_ANY_ACCESS )
	GBIP_IOC_EEPROM_RW = _IOC(_IOC_READ|_IOC_WRITE, MAGIC_NUM, 43, 4*sizeof(unsigned int)),
	GBIP_IOC_PORT_RD = _IOR( MAGIC_NUM, 50, read_write_ioctl_t ),
	GBIP_IOC_PORT_WR = _IOW( MAGIC_NUM, 51, read_write_ioctl_t ),
};
//error code
#define DAS_ERR_NO                         0
#define DAS_ERR_POOL_ALLOC              -101
#define DAS_ERR_IOCTL_CODE              -102
#define DAS_ERR_IOCTL_INSIZE            -103
#define DAS_ERR_IOCTL_OUTSIZE           -104
#define DAS_ERR_NO_FUNCTION             -105
#define DAS_ERR_RUNNING                 -106
#define DAS_ERR_DOUBLE_BUFFER_MODE      -107
#define DAS_ERR_SYNC_MODE               -108
#define DAS_ERR_TRIGGER_SOURCE          -109
#define DAS_ERR_PACER_COUNT             -110
#define DAS_ERR_COUNTER_NUMBER          -111
#define DAS_ERR_COUNTER_MODE            -112
#define DAS_ERR_COUNTER_STATE           -113
#define DAS_ERR_ACCESS_COUNT            -114
#define DAS_ERR_CHANNEL_NUMBER          -115
#define DAS_ERR_CHANNEL_BYTE_COUNT      -116
#define DAS_ERR_CHANNEL_LINE            -117
#define DAS_ERR_PORTIO_BYTE_COUNT       -118
#define DAS_ERR_CHANNEL_GAIN_COUNT      -119
#define DAS_ERR_IO_TIMEOUT              -120
#define DAS_ERR_BURST_MODE              -121
#define DAS_ERR_TRIGGER_MODE            -122
#define DAS_ERR_MISALIGN                -123
#define DAS_ERR_NO_DEVICE		-124
#define DAS_STATUS_PENDING		 101

#define DAS_ERR_MULTI_OPEN              -202
#define DAS_ERR_BUFFER_SETUP            -203
#define DAS_ERR_RESOURCE_SETUP          -204
#endif	
/* _GPIB_IOCTL_H */
