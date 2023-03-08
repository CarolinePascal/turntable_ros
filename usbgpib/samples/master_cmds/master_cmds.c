
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include "conio.h"
#include "Adgpib.h"

#define MAXMSGCNT 40960

int bd= 0, value=0;
int num_ltn = 0;       //number of listener
Addr4882_t paddrs[32]; // primary addresses
Addr4882_t ltns[31];   // listen addresses
char RcvBuffer[MAXMSGCNT + 1];  // Read data buffer
char WBuffer[MAXMSGCNT + 1];  // Read data buffer

int i=0, ud=0;

int chklinests( int ud )
{
	short l;

	if( iblines( ud, &l ) & ERR )
		return -1;
        printf("----------- line status -----------\n");
	printf("EOI|ATN|SRQ|REN|IFC|NRFD|NDAC|DAV\n");
        printf(" %1x | %1x | %1x | %1x | %1x | %1x  | %1x  | %1x |\n",\
        (l>>15)&0x1,(l>>14)&0x1,(l>>13)&0x1,(l>>12)&0x1,(l>>11)&0x1,(l>>10)&0x1,
        (l>>9)&0x1,(l>>8)&0x1);
	printf("-----------------------------------\n");
	return 0;
}

int parse_status( int s )
{
        printf("----------- status -----------\n");
	printf("ERR|TIMO|END|SRQI|RQS|CMPL|LOK|REM\n");
        printf(" %1x | %1x  | %1x | %1x | %1x  | %1x  | %1x | %1x |\n",\
        (s>>15)&0x1,(s>>14)&0x1,(s>>13)&0x1,(s>>12)&0x1,(s>>11)&0x1,(s>>10)&0x1,
        (s>>7)&0x1,(s>>6)&0x1);
	printf("CIC|ATN|TACS|LACS|DTAS|DCAS\n");
        printf(" %1x | %1x | %1x  | %1x  | %1x  | %1x |\n",\
        (s>>5)&0x1,(s>>4)&0x1,(s>>3)&0x1,(s>>2)&0x1,(s>>1)&0x1,(s>>0)&0x1);

	printf("-----------------------------------\n");
	return 0;
}

int main(void)  
{
    int value=0;
    char ppr = 0;
    unsigned char spr=0;

    bd = ibfind("gpib0");
    ibrsc(bd, 1);
    printf("ibrsc: sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcnt);
    ibpad(bd, 0);
    SendIFC(bd);
    if (ibsta & ERR)
    {
       printf("Unable to open board");
       return 1;
    }
    ibsre(bd, 1);
    printf("ibsre:\n");
    parse_status( ibsta );
    ud = ibdev (bd, 1, 0, T10s, 1, 0);
    //ibcmd (0, "A ", 2L);
	//	ibtrg    (ud);
    ibclr (ud);
    usleep(40000);
    //send GET command
    ibtrg(ud);
    printf("GET command:\n");
    parse_status( ibsta );
    usleep(40000);	
    //Device Clear DCL
    WBuffer[0] = 0x14; //DCL
    WBuffer[1] = '\0';
    ibcmd (bd, WBuffer, 2L);
    printf("DCL command:\n");
    parse_status( ibsta );
    usleep(40000);
    ibwait(bd, 0);
    paddrs[0] = 1; 
    paddrs[1] = NOADDR;
    //set device in remote with lockout state
    SetRWLS (bd, paddrs);
    printf("RWLS:\n");
    parse_status( ibsta );
    usleep(30000);
    paddrs[0] = NOADDR; 
    //Enable operations from the front panel of the device
    EnableLocal (bd, paddrs);
    printf("EnableLocal:\n");
    parse_status( ibsta );
    usleep(10000);
    //remote enable
    ibsre(bd, 1); 
    printf("remote enable:\n");
    chklinests( bd );
    //configure the parallel poll response
    //perform a parallel poll
    //bit pattern: 0 1 1 E S D2 D1 D0
    //ist_1+DIO5 : 0 1 1 0 1 1  0  0 = 0x6C	
    ibppc(ud, 0x6c);
    printf("ibppc: sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcnt);
    ibwrt(ud, "ppoll?", 6);
    if (ibsta & ERR)
    {
	printf("write error\n");
    }
    usleep(1000);
    //perform the parallel poll
    ibrpp(ud, &ppr);
    printf("ibrpp: sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcnt);
    printf("parallel poll response: %x\n", ppr);
    
    ibrd(ud, RcvBuffer, 100);
    if (ibsta & ERR)
    {
	printf("read error\n");
    }
    RcvBuffer[ibcnt] = '\0';
    printf("%s\n", RcvBuffer);

    //Disable Parallel poll (bit-4)
    ibppc(ud, 0x70);
    printf("disable pp: sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcnt);

    ibwrt(ud, "spoll?", 6);
    if (ibsta & ERR)
    {
	printf("write error\n");
    }
    //waiting for device requesting service
    ibwait(ud, RQS|TIMO);
    parse_status( ibsta );
    //serial poll response byte
    ibrsp(ud, &spr);
    printf("sp byte: %d\n", spr);
    ibrd(ud, WBuffer, 100);
    if (ibsta & ERR)
    {
	printf("read error\n");
    }
    RcvBuffer[ibcnt] = '\0';
    printf("%s\n", RcvBuffer);
    usleep(100000);
    //TCT
    //pass control to device -- the device become the new CIC
    ibpct(ud);
    printf("ibpct: sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcnt);
    parse_status( ibsta );
    usleep(30000);
    paddrs[0] = NOADDR; 
    ibwait(bd, LACS|TIMO);
    ibrd (bd, RcvBuffer, MAXMSGCNT);
    if (ibsta & ERR)
    {
	printf("read error\n");
    }
    RcvBuffer[ibcnt] = '\0';
    printf("%s\n", RcvBuffer);
    
    printf("\nPress Enter to stop\n");
    getch();
    ibonl (ud, 0);
    ibonl (bd, 0);
    return 0;
}
