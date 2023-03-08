#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>
#include "conio.h"
#include "Adgpib.h"

#define MAXMSGCNT 100

int bd= 0, value=0;
int num_ltn = 0;       //number of listener
Addr4882_t paddrs[32]; // primary addresses
Addr4882_t ltns[31];   // listen addresses
char RcvBuffer[MAXMSGCNT + 1];  // Read data buffer

int main(void)
{
  char board_name[100];
  int vi=0; 

  memset(board_name, '\0', 100);
  sprintf(board_name, "gpib0");
  printf("%s\n", board_name);
  bd=ibfind(board_name );
  printf("bd: %d\n", bd);
  ibrsc(bd, 1);
  ibpad(bd, 0 );
  ibask(bd, IbaPAD, &value );
  printf("pad=%d\n", value);
  ibeot( bd, 1 );
  SendIFC(bd);
 
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

        RcvBuffer[ibcntl] = '\0';
        printf("PA%d_IDN: %s\n", ltns[vi], RcvBuffer);
  }
  ibonl(bd, 0);
  return 0;
}
