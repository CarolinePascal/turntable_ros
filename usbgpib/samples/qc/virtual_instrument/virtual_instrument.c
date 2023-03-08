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

#define ARRAYSIZE  16*1024*1024                 // Size of buffer

char    ReadBuffer[ARRAYSIZE + 1];  // Read data buffer

int main(int argc, char *argv[])  {

    int pad = -1;

    int end = 0;

    int sb;

    int GPIB_Board = -1;

    char GPIB_Desc[10];

    if (argc >= 2) {

        if (sscanf(argv[1], "%d", &GPIB_Board) == 1)
            if ( ! (GPIB_Board >= 0 && GPIB_Board <=30) )
                GPIB_Board = -1;
    }

    if (argc >= 3) {

        if (sscanf(argv[2], "%d", &pad) == 1)
            if ( ! (pad >= 0 && pad <=30) )
                pad = -1;
    }

    if (GPIB_Board == -1) {
        do {
            printf("Input GPIB Board ID (0-30): ");
            if (scanf(" %d", &GPIB_Board) == 0)
                continue;
            if (GPIB_Board >= 0 && GPIB_Board <=30)
                break;
        } while(1);
    };

    if (pad == -1) {
        do {
            printf("Input Instrument GPIB Address (0-30): ");
            if (scanf(" %d", &pad) == 0)
                continue;
            if (pad >= 0 && pad <=30)
                break;
        } while(1);
    }

    putchar('\n');

    printf("GPIB Board ID : %d\n", GPIB_Board);
    printf("Instrument GPIB Address : %d\n", pad);

    sprintf(GPIB_Desc, "gpib%d", GPIB_Board);

    // initialization {
    if (ibfind(GPIB_Desc) == -1) {
        printf("ibfind: ");
        if (iberr == EDVR) {
            printf("%s is not found...", GPIB_Desc);
        }
        goto exit;
    }
    ibrsc(GPIB_Board, 0);
    ibpad(GPIB_Board, pad);
    ibconfig(GPIB_Board, IbcTIMING, T1_DELAY_350ns);
    ibtmo(GPIB_Board, T10s);
    // initialization }    

    do {
        sb = ibwait(GPIB_Board, LACS|TIMO);
        if (ibsta & TIMO) {
            printf("ibwait timeout %04x\n", ibsta);
            continue;
        }
        ibrd (GPIB_Board, ReadBuffer, sizeof(ReadBuffer));

        end = 1;

        /*
        if(ibsta & END) {
            iend = 1;
            printf("END %x\n", ibsta);
            //getchar();
        }
        */
        if (ibsta & ERR)
        {
            if (ibsta & TIMO) {
                if (end == 1)
                    break;
            }
            printf("read error %x\n", ibsta);
        }
	
        printf("read %d bytes...\n", ibcnt);

    } while (!kb_hit());

    ibonl (GPIB_Board, 0);

exit:
    putchar('\n');
    //system("pause");

    return 0;

}
