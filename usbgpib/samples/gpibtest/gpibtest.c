#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>
#include "conio.h"
#include "Adgpib.h"

#define MAXMSGCNT 100

int bd= 0;
int value=0, dev=0;
int num_ltn = 0;       //number of listener
Addr4882_t paddrs[32]; // primary addresses
Addr4882_t ltns[31];   // listen addresses
char RcvBuffer[MAXMSGCNT + 1];  // Read data buffer

int ibnotify (int ud, int mask, GpibNotifyCallback_t Callback, void *RefData);

int rcvHandler(int ud, int thread_sta, int thread_err, int thread_cnt, void *callbackData)
{
	printf("read complete event comes....\n");
        printf("ibrd sts: %x err:%d cnt: %d\n", thread_sta, thread_err, thread_cnt);
	RcvBuffer[thread_cnt] = '\0';
        printf("PA_%d: Read Message: %s\n", *((Addr4882_t *) callbackData), RcvBuffer);
	printf("Press 'Enter' to Exit... \n");
}

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
int main(void)
{
  int vi=0;
  char board_name[100], spr=0;

  memset(board_name, '\0', 100);
  sprintf(board_name, "gpib0");
  printf("%s\n", board_name);
  //board settings
  //open interface
  bd=ibfind(board_name );
  printf("bd: %d\n", bd);
  printf("ibfind sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcntl);
  //set as system controller
  ibrsc(bd, 1);
  printf("ibrsc sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcntl);
  //get system controller status
  ibask(bd, IbaSC, &value );
  printf("sc=%d\n", value); 
  //set primary address
  ibpad(bd, 0 );
  printf("ibpad sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcntl);
  //get primary address
  ibask(bd, IbaPAD, &value );
  printf("pad=%d\n", value);
  //set EOT
  ibeot( bd, 1 );
  printf("ibeot sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcntl);
  //set bus timming as 350ns
  ibconfig( bd, IbcTIMING, T1_DELAY_350ns);
  //set allowing timeout value
  ibtmo(bd, T1s);
  printf("ibtmo sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcntl);
  //get allowing timeout value
  ibask(bd, IbaTMO, &value );
  printf("TMO=%d\n", value);
  //set remote enabled
  ibsre (bd, 1);
  printf("ibsre sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcntl);
  //set autopolling
  ibconfig(bd, IbcAUTOPOLL, 1);
  //set interface clear
  SendIFC(bd);
  printf("SendIFC sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcntl);
  //set cac control by asserting ATN line
  ibcac(bd, 0);
  printf("ibcac sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcntl);  
  //get line status
  chklinests( bd );
  //go to standby by de-asserting ATN line
  ibgts(bd, 0);
  printf("ibgts sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcntl);
  //get line status
  chklinests( bd );
  //Find Listeners
  for (vi = 0; vi < 30; vi++) {
       paddrs[vi] = (Addr4882_t)(vi + 1);
  }
  paddrs[30] = NOADDR;

  printf("\nFinding the listeners ...\n\n");

  FindLstn(bd, paddrs, ltns, 31);
  if (ibsta & ERR)
  {
       printf("FindLstn failed(ibsta: 0x%x)", ibsta);
       return 1;
  }
  num_ltn = ibcntl;
  printf("%d of Listeners found \n\n", num_ltn);

  ltns[num_ltn] = NOADDR;
  //IEEE 488.2 APIs
  //clear devices
  DevClearList(bd, ltns);
  if (ibsta & ERR)
  {
       printf("failed to clear devices");
       return 1;
  }
  //query identification for each listner
  SendList(bd, ltns, "*idn?", 5L, DABend);
  for (vi = 0; vi<num_ltn; vi++)
  {
	Receive(bd, ltns[vi], RcvBuffer, MAXMSGCNT, STOPend);
	if (ibsta & ERR)
        {
              printf("failed to query Identification");
              return 1;
	}

        RcvBuffer[ibcnt] = '\0';
        printf("PA%d_IDN: %s\n", ltns[vi], RcvBuffer);
  }
 
  //receive data with event callback
  for (vi = 0; vi<num_ltn; vi++)
  {
   dev = ibdev (bd, ltns[vi], 0, T1s, 1, 0);

   ibwrt(dev, "*sre 16", 7);
   if (ibsta & ERR)
   {
       printf( "failed to write to devices %x %x", ibsta, iberr);
       return 1;
   }
   printf("ibwrt sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcnt);
   ibwrt(dev, "*idn?", 5); 
   if(ibsta & ERR)
   {
	printf("write error\n");	
   }
   printf("ibwrt: sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcnt);
   ibwait(dev, RQS|TIMO);
   //serial poll status
   ibrsp( dev, &spr );
   printf("ibrsp: spr: %x sts:%x err:%d cnt: %d\n", spr, ibsta, iberr, ibcnt);
   ibrd(dev, RcvBuffer, MAXMSGCNT);   
   printf("ibrd sts: %x err:%d cnt: %d\n", ibsta, iberr, ibcnt);
   RcvBuffer[ibcnt] = '\0';
   printf("PA_%d: Sync_Read Message: %s\n", ltns[vi], RcvBuffer);

   //async send/read
   ibwrta(dev, "*idn?", 5); 
   while (!(ibsta & CMPL))
   {
       ibwait(dev, 0);
       if (ibsta & ERR)
       {
	  printf ("writing device failed : 0x%x.\n", iberr);
          return -1;
       }
   } 
   //read the result
   if(ibrda (dev, RcvBuffer, MAXMSGCNT) & ERR) 
	{
	   printf ("read result failed : 0x%x.\n", iberr);
       return -1;
   }
   while (!(ibsta & END))
   {
      ibwait(dev, 0x000);
      if (ibsta & ERR)
      {
	  printf ("read result failed : 0x%x.\n", iberr);
          return -1;
      }
   } 
   RcvBuffer[ibcnt] = '\0';
   printf("PA_%d: Async_Read Message: %s\n", ltns[vi], RcvBuffer);

   //async read with event callback
   ibwrt(dev, "*idn?", 5); 
   if(ibsta & ERR)
   {
	printf("write error\n");	
   }
   //callback function
   ibnotify(dev, 0x100, rcvHandler, &ltns[vi]);
   ibrda(dev, RcvBuffer, MAXMSGCNT);
   getchar();
   ibwrt(dev,"*sre 0", 6);
  }
  ibonl(bd, 0);

return 0;
}
