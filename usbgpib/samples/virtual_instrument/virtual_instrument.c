
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
#define GPIB_Board      1                // Board handle

#define _GPIB_Desc(ID)  "gpib" #ID

#define GPIB_Desc _GPIB_Desc(GPIB_Board)

char    ReadBuffer[ARRAYSIZE + 1];  // Read data buffer
char    WBuffer[ARRAYSIZE + 1];  // Read data buffer

int main(void)  {

    int	loop=0;                       // Loop counter
    int pad;
    char idn[100];
    int idn_sz=0, res=0;

    do {
        printf("Input GPIB Address(0-30): ");
        if (scanf(" %d", &pad) == 0)
            continue;
        if (pad >= 0 && pad <=30)
            break;
    } while(1);

    do {
        printf("Input GPIB IDN string: ");
        if (scanf(" %99[^\n]%n", idn, &idn_sz) == 0)
            continue;
        break;
    } while(1);

    res = system("pause");
    
    ibfind(GPIB_Desc);
    ibrsc(GPIB_Board, 0);
    ibpad(GPIB_Board, pad);
    ibtmo(GPIB_Board, T10s);
    do {
        do {
            ibwait(GPIB_Board, LACS|TIMO);
            ibrd (GPIB_Board, ReadBuffer, ARRAYSIZE);
            if (ibsta & ERR)
            {
                printf("read error %x\n", ibsta);
                continue;
            }
            if((strncmp (ReadBuffer, "*idn?", 5)) !=0) {
                printf("not idn\n");
                ReadBuffer[ibcntl] = '\0';
                printf("Returned %d string: %s\n", ibcntl, ReadBuffer);
                getchar();
            }
            else
                break;
        } while(1);
        ibwait(GPIB_Board, TACS|TIMO);
        ibwrt(GPIB_Board, idn, idn_sz);
        if (ibsta & ERR)
        {
            printf("%d write error %x\n", loop, ibsta);
        }
    
        loop++;
    } while (!kb_hit());
    ibonl (GPIB_Board, 0);
    res = system("pause");

    return 0;

}
