/***************************************************************************
                          ib.h  -  header file for gpib library
                             -------------------

    copyright            : (C) 2002 by Frank Mori Hess
    email                : fmhess@users.sourceforge.net
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _PUBLIC_GPIB_H
#define _PUBLIC_GPIB_H

#ifdef __cplusplus
extern "C" {
#endif

//#include <stdint.h>
#include "gpib_user.h"

typedef uint16_t Addr4882_t;
typedef int  ssize_t;
typedef unsigned int  size_t;
static const Addr4882_t NOADDR = -1;

/* tells RcvRespMsg() to stop on EOI */
static const int STOPend = 0x100;

enum sad_special_address
{
	NO_SAD = 0,
	ALL_SAD = -1
};

enum send_eotmode
{
	NULLend = 0,
	DABend = 2,
	NLend = 1
};

//extern volatile int ibsta, ibcnt, iberr;
//extern volatile long ibcntl;
#if defined(UNICODE)
   #define ibbna  ibbnaW
   #define ibfind ibfindW
   #define ibrdf  ibrdfW
   #define ibwrtf ibwrtfW
#else
   #define ibbna  ibbnaA
   #define ibfind ibfindA
   #define ibrdf  ibrdfA
   #define ibwrtf ibwrtfA
#endif

typedef int (__stdcall * GpibNotifyCallback_t)(int, int, int, long, void *);

//#pragma comment(linker, "/section:.ASHARE,RWS")
__declspec(dllimport) volatile int ibsta;
__declspec(dllimport) volatile int iberr;
__declspec(dllimport) volatile int ibcnt;
__declspec(dllimport) volatile long ibcntl;
__declspec(dllimport) char CopyrightString[100];

volatile int user_ibsta;
volatile int user_iberr;
volatile int user_ibcnt;
volatile long user_ibcntl;

extern void __stdcall AllSPoll( int board_desc, const Addr4882_t addressList[], short resultList[] );
extern void __stdcall AllSpoll( int board_desc, const Addr4882_t addressList[], short resultList[] );
extern void __stdcall DevClear( int board_desc, Addr4882_t address );
extern void __stdcall DevClearList( int board_desc, const Addr4882_t addressList[] );
extern void __stdcall EnableLocal( int board_desc, const Addr4882_t addressList[] );
extern void __stdcall EnableRemote( int board_desc, const Addr4882_t addressList[] );
extern void __stdcall FindLstn( int board_desc, const Addr4882_t padList[],
	Addr4882_t resultList[], int maxNumResults );
extern void __stdcall FindRQS( int board_desc, const Addr4882_t addressList[], short *result );
extern void __stdcall PassControl( int board_desc, Addr4882_t address );
extern void __stdcall PPoll( int board_desc, short *result );
extern void __stdcall PPollConfig( int board_desc, Addr4882_t address, int dataLine, int lineSense );
extern void __stdcall PPollUnconfig( int board_desc, const Addr4882_t addressList[] );
extern void __stdcall RcvRespMsg( int board_desc, void *buffer, long count, int termination );
extern void __stdcall ReadStatusByte( int board_desc, Addr4882_t address, short *result );
extern void __stdcall Receive( int board_desc, Addr4882_t address,
	void *buffer, long count, int termination );
extern void __stdcall ReceiveSetup( int board_desc, Addr4882_t address );
extern void __stdcall ResetSys( int board_desc, const Addr4882_t addressList[] );
extern void __stdcall Send( int board_desc, Addr4882_t address, const void *buffer,
	long count, int eot_mode );
extern void __stdcall SendCmds( int board_desc, const void *cmds, long count );
extern void __stdcall SendDataBytes( int board_desc, const void *buffer,
	long count, int eotmode );
extern void __stdcall SendIFC( int board_desc );
extern void __stdcall SendLLO( int board_desc );
extern void __stdcall SendList( int board_desc, const Addr4882_t addressList[], const void *buffer,
	long count, int eotmode );
extern void __stdcall SendSetup( int board_desc, const Addr4882_t addressList[] );
extern void __stdcall SetRWLS( int board_desc, const Addr4882_t addressList[] );
extern void __stdcall TestSRQ( int board_desc, short *result );
extern void __stdcall TestSys( int board_desc, const Addr4882_t addressList[],
	short resultList[] );
extern int __stdcall ThreadIbsta( void );
extern int __stdcall ThreadIberr( void );
extern int __stdcall ThreadIbcnt( void );
extern long __stdcall ThreadIbcntl( void );
extern void __stdcall Trigger( int board_desc, Addr4882_t address );
extern void __stdcall TriggerList( int board_desc, const Addr4882_t addressList[] );
extern void __stdcall WaitSRQ( int board_desc, short *result );
extern int __stdcall ibask( int ud, int option, int *value );
extern int __stdcall ibbna( int ud, char *board_name );
extern int __stdcall ibcac( int ud, int synchronous );
extern int __stdcall ibclr( int ud );
extern int __stdcall ibcmd( int ud, const void *cmd, long cnt );
extern int __stdcall ibcmda( int ud, const void *cmd, long cnt );
extern int __stdcall ibconfig( int ud, int option, int value );
extern int __stdcall ibdev( int board_index, int pad, int sad, int timo, int send_eoi, int eosmode );
extern int __stdcall ibdma( int ud, int v );
extern int __stdcall ibeot( int ud, int v );
extern int __stdcall ibeos( int ud, int v );
extern int __stdcall ibevent( int ud, short *event );
extern int __stdcall ibfind( const char *dev );
extern int __stdcall ibgts(int ud, int shadow_handshake);
extern int __stdcall ibist( int ud, int ist );
extern int __stdcall iblines( int ud, short *line_status );
extern int __stdcall ibln( int ud, int pad, int sad, short *found_listener );
extern int __stdcall ibloc( int ud );
extern int __stdcall ibonl( int ud, int onl );
extern int __stdcall ibpad( int ud, int v );
extern int __stdcall ibpct( int ud );
extern int __stdcall ibppc( int ud, int v );
extern int __stdcall ibrd( int ud, void *buf, long count );
extern int __stdcall ibrda( int ud, void *buf, long count );
extern int __stdcall ibrdf( int ud, const char *file_path );
extern int __stdcall ibrpp( int ud, char *ppr );
extern int __stdcall ibrsc( int ud, int v );
extern int __stdcall ibrsp( int ud, char *spr );
extern int __stdcall ibrsv( int ud, int v );
extern int __stdcall ibsad( int ud, int v );
extern int __stdcall ibsic( int ud );
extern int __stdcall ibsre( int ud, int v );
extern int __stdcall ibstop( int ud );
extern int __stdcall ibtmo( int ud, int v );
extern int __stdcall ibtrg( int ud );
extern int __stdcall ibwait( int ud, int mask );
extern int __stdcall ibwrt( int ud, const void *buf, long count );
extern int __stdcall ibwrta( int ud, const void *buf, long count );
extern int __stdcall ibwrtf( int ud, const char *file_path );
extern const char* gpib_error_string( int iberr );

extern int __stdcall ibnotify (int ud, int mask, GpibNotifyCallback_t Callback, void *RefData);
extern int __stdcall ibpoke   (int ud, long option, long v);


static __inline Addr4882_t MakeAddr( unsigned int pad, unsigned int sad )
{
	Addr4882_t address;

	address = ( pad & 0xff );
	address |= ( sad << 8 ) & 0xff00;
	return address;
}

static __inline unsigned int GetPAD( Addr4882_t address )
{
	return address & 0xff;
}

static __inline unsigned int GetSAD( Addr4882_t address )
{
	return ( address >> 8 ) & 0xff;
}

#ifdef __cplusplus
}
#endif

#endif	/* _PUBLIC_GPIB_H */
