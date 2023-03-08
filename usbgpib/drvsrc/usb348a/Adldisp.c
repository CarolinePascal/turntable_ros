#include "../include/Adldev.h"
#include "../include/Adllib.h"

/*---- Local function in other module ----*/
extern void PLX90_ClearController(U32 dwPort);

/*===========================================================*/
/* PCI Controller Reset */
void AdlControllerReset( PPCI_DEVEXT  pDevExt, U32 reset_9050)   //Alex 12/09/98
{
       PLX90_ClearController(OP_Port(pDevExt));  //Alex 12/02/98

}
/*===========================================================*/
/* Open Device assist func */
BOOLEAN AdlOpenDevice( PPCI_DEVEXT  pDevExt )
{

        /*-------------------------------------*/
        /* error check */
        if (! pDevExt )
                return FALSE;
        if ( pDevExt->Reference > 0 )	{
         	return TRUE;
	}
        /*-------------------------------------*/
        /* PCI Controller Reset */
        AdlControllerReset( pDevExt, 1 );
        /*-------------------------------------*/
        pDevExt->Reference = 0;
        return TRUE;

}


















