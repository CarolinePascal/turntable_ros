/***************************************************************************
 gpib_user.h  -  constant definition
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from include/gpib_user.h of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/
 
#ifndef _GPIB_USER_H
#define _GPIB_USER_H

#define GPIB_MAX_NUM_BOARDS 16
#define GPIB_MAX_NUM_DESCRIPTORS 0x1000

typedef unsigned char   U8;
typedef short           I16;
typedef unsigned short  U16;
typedef int             I32;
typedef unsigned int    U32;
typedef long            I64;
typedef unsigned long   U64;
typedef float           F32;
typedef double          F64;

enum ibsta_bit_numbers
{
	DCAS_NUM = 0,
	DTAS_NUM = 1,
	LACS_NUM = 2,
	TACS_NUM = 3,
	ATN_NUM = 4,
	CIC_NUM = 5,
	REM_NUM = 6,
	LOK_NUM = 7,
	CMPL_NUM = 8,
	RQS_NUM = 11,
	SRQI_NUM = 12,
	END_NUM = 13,
	TIMO_NUM = 14,
	ERR_NUM = 15
};

/* IBSTA status bits */
enum ibsta_bits
{
	DCAS = ( 1 << DCAS_NUM ),	/* device clear state */
	DTAS = ( 1 << DTAS_NUM ),	/* device trigger state */
	LACS = ( 1 <<  LACS_NUM ),	/* Listener active */
	TACS = ( 1 <<  TACS_NUM ),	/* Talker active */
	ATN = ( 1 <<  ATN_NUM ),	/* Attention asserted */
	CIC = ( 1 <<  CIC_NUM ),	/* Controller-in-Charge */
	REM = ( 1 << REM_NUM ),		/* remote state */
	LOK = ( 1 << LOK_NUM ),		/* lockout state */
	CMPL = ( 1 <<  CMPL_NUM ),	/* I/O completed  */
	RQS = ( 1 <<  RQS_NUM ),	/* Device requests service  */
	SRQI = ( 1 << SRQI_NUM ),	/* SRQ asserted */
	END = ( 1 << END_NUM ),		/* EOI or EOS */
	TIMO = ( 1 << TIMO_NUM ),	/* Time interval exceeded */
	ERR = ( 1 << ERR_NUM )		/* error occured */
};
/* IBERR error codes */
enum iberr_code
{
	EDVR = 0,		/* system error */
	ECIC = 1,		/* not CIC */
	ENOL = 2,		/* no listeners */
	EADR = 3,		/* addressed not correctly */
	EARG = 4,		/* invalid argument */
	ESAC = 5,		/* not SAC */
	EABO = 6,		/* I/O was aborted */
	ENEB = 7,		/* board is not existed */
	EDMA = 8,		/* DMA error detected */
	EOIP = 10,		/* previous I/O in progress  */
	ECAP = 11,		/* operation not supported */
	EFSO = 12,		/* file system operation error */
	EBUS = 14,		/* bus error */
	ESTB = 15,		/* serial poll byte lost*/
	ESRQ = 16,		/* SRQ keeps asserted */
	ETAB = 20               /* Table Overflow */
};

//Timeout values

enum gpib_timeout
{
	TNONE = 0,		/* Infinite timeout (disabled)    */
	T10us = 1,		/* timeout:  10 usec  (ideal)     */
	T30us = 2,		/* Timeout:  30 usec  (ideal)     */
	T100us = 3,		/* Timeout:  100 usec (ideal)     */
	T300us = 4,		/* Timeout:  300 usec (ideal)     */
	T1ms = 5,		/* Timeout:  1 msec (ideal)       */
	T3ms = 6,		/* Timeout:  3 msec (ideal)       */
	T10ms = 7,		/* Timeout:  10 msec (ideal)      */
	T30ms = 8,		/* Timeout:  30 msec (ideal)      */
	T100ms = 9,		/* Timeout:  100 msec (ideal)     */
	T300ms = 10,		/* Timeout:  300 msec (ideal)     */
	T1s = 11,		/* Timeout:  1 sec (ideal)        */
	T3s = 12,		/* Timeout:  3 sec (ideal)        */
	T10s = 13,		/* Timeout:  10 sec (ideal)       */
	T30s = 14,		/* Timeout:  30 sec (ideal)       */
	T100s = 15,		/* Timeout:  100 sec (ideal)      */
	T300s = 16,		/* Timeout:  300 sec (ideal)      */
	T1000s = 17		/* Timeout:  1000 sec (ideal)   */
};

//EOS mode

enum eos_flags
{
	EOS_MASK = 0x1c00, /* mask */
	REOS = 0x0400,	/* stop to read on EOS	*/
	XEOS = 0x800,	/* EOI asserted with EOS    */
	BIN = 0x1000	/* 8-bit comparision on EOS   */
};

// GPIB Bus Control Lines bits
enum bus_control_line
{
	ValidDAV = 0x01,
	ValidNDAC = 0x02,
	ValidNRFD = 0x04,
	ValidIFC = 0x08,
	ValidREN = 0x10,
	ValidSRQ = 0x20,
	ValidATN = 0x40,
	ValidEOI = 0x80,
	ValidALL = 0xff,
	BusDAV = 0x0100,		/* DAV  status */
	BusNDAC = 0x0200,		/* NDAC status */
	BusNRFD = 0x0400,		/* NRFD status */
	BusIFC = 0x0800,		/* IFC  status */
	BusREN = 0x1000,		/* REN  status */
	BusSRQ = 0x2000,		/* SRQ  status */
	BusATN = 0x4000,		/* ATN  status */
	BusEOI = 0x8000			/* EOI  status */
};


static const int gpib_addr_max = 30;

//ibask items
enum ibask_option
{
	IbaPAD = 0x1,
	IbaSAD = 0x2,
	IbaTMO = 0x3,
	IbaEOT = 0x4,
	IbaPPC = 0x5,		/* ppoll configuration: board only */
	IbaREADDR = 0x6,	/* repeat addressing: device only */
	IbaAUTOPOLL = 0x7,	/* autopolling: board only */
	IbaCICPROT = 0x8,	/* CIC protocol: board only */
	IbaIRQ = 0x9,		/* IRQ: board only */
	IbaSC = 0xa,		/* system controller: board only */
	IbaSRE = 0xb,		/* remote enabled: board only */
	IbaEOSrd = 0xc,		/* EOS on read */
	IbaEOSwrt = 0xd,	/* EOS on write */
	IbaEOScmp = 0xe,	/* 7 or 8 bits of EOS comparision */
	IbaEOSchar = 0xf,	/* EOS character */
	IbaPP2 = 0x10,		/* PPoll mode 2: board only */
	IbaTIMING = 0x11,	/* bus timming (t1): board only */
	IbaDMA = 0x12,		/* Dma: board only */
	IbaReadAdjust = 0x13,   /* swap read bytes: not supported */
	IbaWriteAdjust = 0x14,  /* swap write bytes: not supported */
	IbaSendLLO = 0x17,	/* sending LLO: board only */
	IbaSPollTime = 0x18,	/* spoll timeout: device only */
	IbaPPollTime = 0x19,	/* ppoll timeout: board only */
	IbaEndBitIsNormal = 0x1a, /* not supported */
	IbaUnAddr = 0x1b,	/* device unadressing: not supported( device only )*/
	IbaHSCableLength = 0x1f,	/* cable length: not supported (board only) */
	IbaIst = 0x20,	/* IST bit: board only */
	IbaRsv = 0x21,	/* Rsv byte: board only */
	IbaBNA = 0x200,	/* board associated with the device: device only */
    	IbaBaseAddr = 0x201	/* GPIB board's base I/O address: board only*/
};

//ibconfig items
enum ibconfig_option
{
	IbcPAD = 0x1,
	IbcSAD = 0x2,
	IbcTMO = 0x3,
	IbcEOT = 0x4,
	IbcPPC = 0x5,	/* board only */
	IbcREADDR = 0x6,	/* device only */
	IbcAUTOPOLL = 0x7,	/* board only */
	IbcCICPROT = 0x8,	/* board only */
	IbcIRQ = 0x9,	/* board only */
	IbcSC = 0xa,	/* board only */
	IbcSRE = 0xb,	/* board only */
	IbcEOSrd = 0xc,
	IbcEOSwrt = 0xd,
	IbcEOScmp = 0xe,
	IbcEOSchar = 0xf,
	IbcPP2 = 0x10,	/* board only */
	IbcTIMING = 0x11,	/* board only */
	IbcDMA = 0x12,	/* board only */
	IbcReadAdjust = 0x13,
	IbcWriteAdjust = 0x14,
	IbcEventQueue = 0x15,	/* board only */
	IbcSPollBit = 0x16,	/* board only */
	IbcSpollBit = 0x16,	/* board only */
	IbcSendLLO = 0x17,	/* board only */
	IbcSPollTime = 0x18,	/* device only */
	IbcPPollTime = 0x19,	/* board only */
	IbcEndBitIsNormal = 0x1a,
	IbcUnAddr = 0x1b,	/* device only */
	IbcHSCableLength = 0x1f,	/* board only */
	IbcIst = 0x20,	/* board only */
	IbcRsv = 0x21,	/* board only */
	IbcLON = 0x22,
	IbcBNA = 0x200	/* device only */
};

//t1 value
enum t1_delays
{
	T1_DELAY_2000ns = 1,
	T1_DELAY_500ns = 2,
	T1_DELAY_350ns = 3
};

#endif	/* _GPIB_USER_H */
