#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include "conio.h"
#include "Adgpib.h"

#define ARRAYSIZE  40960                 // Size of read buffer
#define GPIB_Board     0                   // Board handle

#define _GPIB_Desc(ID)  "gpib" #ID

#define GPIB_Desc _GPIB_Desc(GPIB_Board)

char    ReadBuffer[ARRAYSIZE + 1];  // Read data buffer
char    WBuffer[ARRAYSIZE + 1];  // Read data buffer

int main(void)  {

    int i=0, ud=0;
    int res = 0;

    int pad;

    do {
        printf("Input Instrument GPIB Address(0-30): ");
        fflush(stdin);
        if (scanf("%d", &pad) == 0)
            continue;
        if (pad >= 0 && pad <=30)
            break;
    } while(1);

    ibfind(GPIB_Desc);
    ibrsc(GPIB_Board, 1);
    ibpad(GPIB_Board, 0);
    SendIFC(GPIB_Board);
    if (ibsta & ERR)
    {
       printf("Unable to open board\n");
       return 1;
    }
    ibsre(GPIB_Board, 1);
    ud = ibdev (GPIB_Board, pad, 0, T10s, 1, 0);
    do {

        ibwrt(ud, "*idn?", 5);
        if (ibsta & ERR)
        {
            printf("write error\n");
        }
        printf("send %d data finish\n", ibcntl);
        i++;
        ibrd(ud, WBuffer, 100);
        WBuffer[ibcntl]='\0';
        printf("rd: %s\n", WBuffer);
        usleep(1000);
    } while (!kb_hit());
    printf("kbhit\n");
    ibonl (ud, 0);
    ibonl (GPIB_Board, 0);
    res = system("pause");
 
    return 0;
}
