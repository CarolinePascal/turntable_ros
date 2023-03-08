
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

	ibfind("gpib0");
	ibrsc(bd, 0);
	ibpad(bd,1);
	ibtmo(bd, T30s);
        printf("waiting for device clear...\n");
	ibwait(bd, DCAS|TIMO);
	parse_status( ibsta );
	ibtmo(bd, T3s);
        printf("waiting for GET...\n");
	ibwait(bd, DTAS|TIMO);
	parse_status( ibsta );
        printf("waiting for DCL...\n");
	ibwait(bd, DCAS|TIMO);
	parse_status( ibsta );
        printf("waiting for LOK...\n");
	ibwait(bd, LOK|TIMO);
	parse_status( ibsta );
	do {
		ibwait(bd, 0);
		usleep(10000);
	} while(ibsta & LOK);
//ppoll
        printf("waiting for LACS...\n");
	ibwait(bd, LACS|TIMO);
	parse_status( ibsta );
	ibrd (bd, RcvBuffer, MAXMSGCNT);
        if (ibsta & ERR)
        {
		printf("read error\n");
        }
	RcvBuffer[ibcnt] = '\0';
        printf("%s\n", RcvBuffer);
        //responds pp
	ibist(bd, 1);
        printf("waiting for TACS...\n");
	ibwait(bd, TACS|TIMO);
	parse_status( ibsta );
	ibwrt(bd, "USB348A PPOLL", 13);
	if (ibsta & ERR)
        {
		printf("write error\n");
        }
//spoll
	ibwait(bd, LACS|TIMO);
	ibrd (bd, RcvBuffer, MAXMSGCNT);
        if (ibsta & ERR)
        {
		printf("read error\n");
        }
	RcvBuffer[ibcnt] = '\0';
        printf("%s\n", RcvBuffer);
	printf("Writing spoll register %x and raise the SRQ signal\n", 0x41);
	ibrsv(bd, 0x41);   //write spoll register and raise the SRQ signal
	ibwait(bd, TACS|TIMO);
	ibwrt(bd, "USB348A SPOLL", 13);
//TCT
	printf("Waiting for CIC pass control...\n");
	ibwait(bd, CIC|TIMO);
	parse_status( ibsta );
	//set talker and lisener
	printf("set Talker(PA: 1) and Listener(PA: 0)\n");
	ibcmd(bd, "A ", 2);
	parse_status( ibsta );
	ibwrt(bd, "TCT test", 8);
        if (ibsta & ERR)
        {
		printf("write error\n");
		goto end_s;
        }
	printf("Return to local...\n");
	ibloc(bd); //return to local
	do {unsigned short l=0;
		ibwait(bd, 0);
		iblines(bd, &l);
		printf("line: %x\n", l);
	} while(ibsta & REM);
	parse_status( ibsta );
end_s:
    ibonl (bd, 0);
printf("press any key to stop\n");
	getch();
    return 0;

}
