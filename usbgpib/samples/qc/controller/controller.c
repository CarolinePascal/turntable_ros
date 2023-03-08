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

char    WBuffer[ARRAYSIZE + 1];  // Read data buffer

typedef struct st_results {
    int bytes;
    double rate;
} st_results;

int main(int argc, char *argv[])  {

    int ud = 0;

    int pad = -1;

    int i, j, k, x, y;

    double take_msec, timeout_u, rt;

    struct timeval tv_start, tv_end, tv_res;
    struct timezone tz;
    
    int GPIB_Board = -1;

    char GPIB_Desc[10];


    int loops = 0;

    st_results results[20];

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

    if (argc >= 4) {

        if (sscanf(argv[3], "%d", &loops) == 1)
            if ( ! (loops > 0) )
                loops = 0;
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
            fflush(stdin);
            if (scanf("%d", &pad) == 0)
                continue;
            if (pad >= 0 && pad <=30)
                break;
        } while(1);
    }

    if (loops == 0) {
        do {
            printf("Input Run loop count : ");
            if (scanf("%d", &loops) == 0)
                continue;
            if (loops > 0)
                break;
        } while(1);
    }

    putchar('\n');

    printf("GPIB Board ID : %d\n", GPIB_Board);
    printf("Instrument GPIB Address : %d\n", pad);
    printf("run loops : %d\n", loops);

    sprintf(GPIB_Desc, "gpib%d", GPIB_Board);

    // initialization {
    if (ibfind(GPIB_Desc) == -1) {
        printf("ibfind: ");
        if (iberr == EDVR) {
            printf("%s is not found...", GPIB_Desc);
        }
        goto exit;
    }
    ibrsc(GPIB_Board, 1);
    ibpad(GPIB_Board, 0);
    ibeot(GPIB_Board, 1);
    ibconfig(GPIB_Board, IbcTIMING, T1_DELAY_350ns);    
    ibtmo(GPIB_Board, T10s);    
    SendIFC(GPIB_Board);
    if (ibsta & ERR)
    {
       printf("Unable to open board\n");
       return 1;
    }
    ibsre(GPIB_Board, 1);
    ud = ibdev (GPIB_Board, pad, 0, T10s, 1, 0);
    if (ud == -1) {
        printf("ibdev: ");
        if (iberr == EDVR) {
            printf("%s is not found...", GPIB_Desc);
        }
        goto exit2;
    }

    for(y = 0; y < sizeof(results)/sizeof(results[0]); ++y) {
        results[y].bytes = 0;
        results[y].rate = 0.0;
    }

    // initialization }

       
    for (x = 0; x < loops; ++x) {

        puts("\n===========================================================");
        printf("\nloops %d ====>>\n", x+1);

        for(j = 1, y = 0; j <= ARRAYSIZE && y < sizeof(results)/sizeof(results[0]); j *= 10, ++y) {
            k = j;
            for(i = 0; i < j; ++i, ++k) {
                WBuffer[i] = k;
            }
            WBuffer[j] = '\0';
            
    	    gettimeofday( &tv_start, &tz );

            ibwrt (ud, WBuffer, j);

            while(!(ibsta&CMPL)) {
            }

            gettimeofday( &tv_end, &tz );

            if (ibsta & ERR)
            {
                printf("write ERR %x\n", ibsta);
                //goto exit;
            }
	     
	    timersub(&tv_end, &tv_start, &tv_res);
	    take_msec = (double) ((tv_res.tv_sec*1000)+(tv_res.tv_usec/1000));
	    rt = ((double) ibcntl)/ take_msec;
            printf("\n%9d (%9d) : %9.3f msec => %9.3f KByte/s\n", ibcntl, j, take_msec, rt);

#if 0
            take_msec =((double)(current_count.QuadPart-start_count.QuadPart))/freq.QuadPart*1000;
            rt = ((double) ibcntl)/take_msec;
            printf("\n%9d (%9d) : %9.3f msec => %9.3f KByte/s\n", ibcntl, j, take_msec, rt);
#endif
            results[y].bytes = j;
            results[y].rate += rt;

            if (ibcnt != j)
                goto exit3;

        }

    }

    puts("\n===========================================================");

    printf("\naverage of %d loops ====>>\n\n", loops);

    for(y = 0; y < sizeof(results)/sizeof(results[0]); ++y) {
        if (results[y].bytes == 0)
            break;
        results[y].rate /= loops;
        printf("%9d : %9.3f KByte/s\n", results[y].bytes, results[y].rate);
    }

exit3:
    ibonl (ud, 0);
exit2:
    ibonl (GPIB_Board, 0);
exit:
    putchar('\n');
    //system("pause");
 
    return 0;
}
