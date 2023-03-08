
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>
#include "conio.h"
#include "Adgpib.h"

#define MAXMSGCNT 40960
#define GPIB0        0                 // Board handle
#define GPIB1        1                 // Board handle

int bd= 0, value=0;
int  vi=0;
char RcvBuffer[MAXMSGCNT + 1];  // Read data buffer
char WBuffer[MAXMSGCNT + 1];  // Read data buffer

char tx_data[] = "1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!1234567890 abcdefghijklmnopqrstuvwxyz!!!";

int main(void)  {
    double take_msec;
    struct timeval tv_start, tv_end, tv_res;
    struct timezone tz;

    ibfind("gpib0");
    ibrsc(GPIB0, 1);
    ibpad(GPIB0, 0);
    ibconfig(GPIB0, IbcTIMING, T1_DELAY_350ns);
    ibfind("gpib1");
    ibrsc(GPIB1, 0);
    ibpad(GPIB1, 1);
    ibconfig(GPIB1, IbcTIMING, T1_DELAY_350ns);

    SendIFC(GPIB0);
    if (ibsta & ERR)
    {
       printf("Unable to open board");
       return 1;
    }
    sprintf(WBuffer, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s", tx_data, tx_data,  tx_data, tx_data,  tx_data, tx_data, tx_data,
		tx_data, tx_data,  tx_data, tx_data,  tx_data, tx_data, tx_data, tx_data, tx_data,  tx_data, tx_data,  tx_data, tx_data, tx_data,
		tx_data, tx_data,  tx_data, tx_data,  tx_data, tx_data, tx_data, tx_data, tx_data,  tx_data, tx_data,  tx_data, tx_data, tx_data,
		tx_data, tx_data,  tx_data, tx_data,  tx_data, tx_data, tx_data, tx_data, tx_data,  tx_data, tx_data,  tx_data, tx_data, tx_data,
		tx_data, tx_data,  tx_data, tx_data,  tx_data, tx_data, tx_data, tx_data, tx_data);
  WBuffer[strlen(WBuffer)] = '\0';
  ibcmd (GPIB0, "@!", 2L);   //master talk and slave listen
  //ibcmd (GPIB0, "A ", 2L); //slave talk and master listen
  if (ibsta & ERR)
  {
       printf("Unable to send command");
       return 1;
  }
  do {
    //ibrda (0, ReadBuffer, ARRAYSIZE);
     gettimeofday( &tv_start, &tz );
     ibwrta(GPIB0, WBuffer, strlen(WBuffer));
     ibrd (GPIB1, RcvBuffer, strlen(WBuffer));
     //ibwrt(GPIB1, WBuffer, strlen(WBuffer));
     //printf(" Start time is: %ld.%06ld sec\n", tv_start.tv_sec, tv_start.tv_usec);
     gettimeofday( &tv_end, &tz );
     ibwait(GPIB0, 0);
     //gettimeofday( &tv_end, &tz );
     //printf(" (%x) End time is: %ld.%06ld sec\n", ibsta, tv_end.tv_sec, tv_end.tv_usec);

     if(ibsta &ERR) {
	   printf("w error: %x %x\n", ibsta, ibcntl);
 	   gettimeofday( &tv_end, &tz );
	   return 1;
     }

     while(!(ibsta&CMPL)) {
		ibwait(GPIB0, 0);
		if(ibsta &ERR) {
		  printf("w error: %x %x\n", ibsta, ibcntl);
 	          gettimeofday( &tv_end, &tz );
                  printf(" End time is: %ld.%06ld sec\n", tv_end.tv_sec, tv_end.tv_usec);
		  return 1;
		}
     }
     RcvBuffer[ibcntl] = '\0';
     gettimeofday( &tv_end, &tz );
     //printf(" End time is: %ld.%06ld sec\n", tv_end.tv_sec, tv_end.tv_usec);
     timersub(&tv_end, &tv_start, &tv_res);
     take_msec = (double) ((tv_res.tv_sec*1000)+(tv_res.tv_usec/1000));
     printf("\ncnt_%d: %9.3f msec\n", ibcnt, take_msec);
     RcvBuffer[ibcntl] = '\0';
     //printf("(%x) cnt: %d\n", ibsta, ibcntl);
     for(vi=0;vi<ibcntl;vi++) {
	   if(RcvBuffer[vi]!=WBuffer[vi]) {
	     printf("%d %c %c\n", vi, WBuffer[vi], RcvBuffer[vi]);
	     printf("%d %c %c\n", vi+1, WBuffer[vi+1], RcvBuffer[vi+1]);
	     printf("%d %c %c\n", vi+2, WBuffer[vi+2], RcvBuffer[vi+2]);
		return 1;
	   }
    }
  } while (!kb_hit());	

    /*  Take the board offline.*/
    ibonl (GPIB1, 0);
    ibonl (GPIB0, 0);
    getchar();
    return 0;

}
