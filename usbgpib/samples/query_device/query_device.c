#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include "conio.h"
#include "Adgpib.h"

#define RSIZE 256            // Size of read buffer
#define BOARD_DESC  0		     // Board Index

int  ud;              // Device handle
char wrtbuf[RSIZE + 1];  // R/W buffer
char buf[RSIZE + 1];  // R/W buffer
int  dev_paddr = 0;
int  dev_saddr = 0;
int  asynio=0, result=0;
char spr=0;

int prompt_for_IDN(int ud)
{    
    if(ibwrt (ud, "*idn?", 5L) & ERR)
    {
	   printf ("writing device failed : 0x%x.\n", iberr);
	   ibonl (ud, 0);
       return 1;
    }
     if(ibrd (ud, buf, RSIZE) & ERR)
    {
	   printf ("reading from device failed : 0x%x.\n", iberr);
	   ibonl (ud, 0);
       return 1;
    }
    buf[ibcntl] = '\0';
    printf("\nDevice IDN: %s\n\n", buf);
	return 0;
}

int async_read_write_test(int ud)
{
    //printf("\nAync. mode of R/W ...\n");

    if( ibwrta(ud, wrtbuf, strlen(wrtbuf)) & ERR )
    {
       printf ("write device failed : 0x%x(%x).\n", ibsta, iberr);
       return -1;
    }

    while (!(ibsta & CMPL))
    {
       ibwait(ud, 0);
       if (ibsta & ERR)
       {
	  printf ("writing device failed : 0x%x.\n", iberr);
          return -1;
       }
    } 
#if 0
    /**** if SRQ enabled ****/
    ibwait(ud, RQS|TIMO);
    if(ibsta & RQS)
    {
	ibrsp(ud, &spr);
	printf("spr: %x\n", spr);
    }
    /************************/
#endif
    //read the result
    if(ibrda (ud, buf, RSIZE) & ERR) 
	{
	   printf ("read result failed : 0x%x.\n", iberr);
       return -1;
    }
    while (!(ibsta & END))
    {
       ibwait(ud, 0x000);
       if (ibsta & ERR)
       {
		  printf ("read result failed : 0x%x.\n", iberr);
          return -1;
       }
    } 

    buf[ibcntl] = '\0';
    printf("\nread result: %s", buf);
    return 0;
}

int sync_read_write_test(int ud)
{
    printf("\nSync. mode of R/W ...\n");

    if( ibwrt(ud, wrtbuf, strlen(wrtbuf)) & ERR )
    {	
	  printf ("writing device failed : 0x%x.\n", iberr);
          return -1;
    }
#if 0
    /** If SRQ is enabled **/
    ibwait(ud, RQS|TIMO);
    if(ibsta & RQS)
    { 
    	ibrsp(ud, &spr);
     	printf("spr: %x\n", spr);
    }
    /************************/
#endif
    //read the result
    if( ibrd (ud, buf, RSIZE) & ERR ) {
	  printf ("writing device failed : 0x%x.\n", iberr);
          return -1;
    }
    buf[ibcntl] = '\0';
    printf("\nread result: %s", buf);
    return 0;
}

int main(void)  {

    printf("This program demonstrates GPIB R/W operation. \n\n");

    printf("Please input the primary address of your device: ");
    scanf(" %d", &dev_paddr);    
    //open devive
    ud = ibdev (BOARD_DESC, dev_paddr, dev_saddr, T3s, 1, 0);
    if (ibsta & ERR)
    {
       printf ("open device failed : 0x%x.\n", iberr);
       return 1;
    }
    printf("ibdev: %d\n", ud);
    //clear device
    //ibclr(ud);
    if (ibsta & ERR)
    {
       printf ("clear device failed: 0x%x.\n", iberr);
       ibonl (ud, 0);
       return 1;
    }
    //Enable SRQ
    //ibwrt(ud, "*sre 16", 7);
    result = prompt_for_IDN(ud);
    if(result) {
	ibonl (ud, 0);
	return 1;
    }

    printf("SYNC(0) or ASYNC(1) read/write? ");
    scanf(" %d", &asynio);
    printf("\nenter a string to send to your device: \n");
    scanf(" %s", wrtbuf);
    printf("\nPress any key to stop... \n");
    do {
	if(asynio)
	   result = async_read_write_test (ud);
	else
	   result = sync_read_write_test (ud);	
    } while (!kb_hit());
    //Disable SRQ
    //ibwrt(ud, "*sre 0", 6);
    //taking device offline.
    ibonl (ud, 0);
	
    return result;
}
