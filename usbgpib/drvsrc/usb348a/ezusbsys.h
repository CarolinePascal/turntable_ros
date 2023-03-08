//////////////////////////////////////////////////////////////////////
//
// File:      ezusbsys.h
// $Archive: /EzUsb/Drivers/ezusbdrv/ezusbsys.h $
//
// Purpose:
//    Header file for the Ezusb USB Device Driver
//
// Environment:
//    kernel mode
//
// $Author: Markm $
//
//////////////////////////////////////////////////////////////////////

#ifndef _BYTE_DEFINED
#define _BYTE_DEFINED
typedef unsigned char BYTE;
#endif // !_BYTE_DEFINED

#ifndef _WORD_DEFINED
#define _WORD_DEFINED
typedef unsigned short WORD;
#endif // !_WORD_DEFINED

#define DIRECTION_OUT 	0 
#define DIRECTION_IN  	1 

enum VR_REQUEST
{
    VR_GPIB_RESPONSE		= 0xB0,
    VR_GPIB_REG			= 0xB1,
    VR_GPIB_2NDADDR		= 0xB2,
    VR_GPIB_CTRL_M		= 0xB3,
    VR_GPIB_SIC			= 0xB4,
    VR_GPIB_RSC			= 0xB5,
    VR_GPIB_CAC			= 0xB6,
    VR_GPIB_GTS			= 0xB7,
    VR_GPIB_CTRL_PP		= 0xB8, 
    VR_GPIB_DO		= 0xB9,
    VR_GPIB_DI		= 0xBA,
    VR_GPIB_ABORT,    
    VR_GPIB_CTRL_SP,    
    VR_GPIB_SREG	= 0xBD,   
    VR_GPIB_STATUS = 0xBE,    
    VR_GPIB_LISTENER,
    VR_GPIB_TIMEOUT = 0xC0,
    VR_GPIB_CLEAR = 0xC1,
    VR_GPIB_PDO,    
    VR_GPIB_PDI,
};

enum VR_RESPONSE 
{
    IDLE = 0,
    BUSY,
    DONE,
};

enum rtn_code{
    rtn_ATN = 0,
    rtn_nATN = 1,
    rtn_NDAC,
    rtn_Err,
    rtn_Abort,
    rtn_Timeout,
    rtn_nCIC,//yuan add 03/17/10
    rtn_Success = 0xff,
};
///////////////////////////////////////////////////////////
//
// control structure for bulk and interrupt data transfers
//
///////////////////////////////////////////////////////////
typedef struct _VENDOR_REQUEST_IN
{
    BYTE     bmRequestType;
    BYTE     bRequest;
    BYTE    wValueL;
    BYTE    wValueH;
    BYTE    wIndexL;
    BYTE    wIndexH;
    BYTE    wLengthL;
    BYTE    wLengthH;
    BYTE     direction;
    BYTE     bData;
} VENDOR_REQUEST_IN, *PVENDOR_REQUEST_IN;

typedef struct _BULK_TRANSFER_CONTROL
{
   unsigned int ep;
   unsigned int pipeNum;
} BULK_TRANSFER_CONTROL, *PBULK_TRANSFER_CONTROL;

   
int Ezusb_ReadWrite(
   U348A_DEVEXT *dev,
   BULK_TRANSFER_CONTROL *bulkControl,
   uint8_t * bulkbuffer,   
   size_t bufferLength,
   int dir,   
   size_t *actualLength,
   int *end
   );


int UDServiceFire(
    U348A_DEVEXT *pDevExt,
    uint8_t bRequest,
    uint16_t wValue,
    uint16_t wIndex,
    int nLen,
    void *buf,
    uint8_t In_direction );
