#ifndef		_ADLP9030_H_
#define		_ADLP9030_H_

#include "Adlplx90.h"

void  P9030_SetInt(void __iomem *dwPort, unsigned short wPin, unsigned short wEnable, unsigned short act_lev, unsigned short trig, unsigned short int_clr);
void  P9030_SetUserIoPin(void __iomem *dwPort, unsigned int wPin, unsigned int wDat);
void  P9030_GetUserIoPin(void __iomem *dwPort, unsigned int wPin, unsigned int* wpDat);
void P9030_ClearController(void __iomem *dwPort);

short READ_EEPROM( void __iomem *ba, unsigned int adr, short* data );
short WRITE_EEPROM( void __iomem *ba, unsigned int adr, unsigned short data );
short WRITE_EEPROM_ENABLE( void __iomem *ba );
short WRITE_EEPROM_DISABLE( void __iomem *ba );

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define		ADL_OP_GPIOC			0x54	  // Added in PLX 9030
#define		ADL_OP_PMDATASEL		0x70      // Added in PLX 9030
#define		ADL_OP_PMDATASCALE		0x74      // Added in PLX 9030
/*--------------------------------------------------------------------------*/
typedef	struct
{
	unsigned short	Int1Enable : 1;
	unsigned short	Int1ActHigh : 1;
	unsigned short	Int1Status : 1;
	unsigned short	Int2Enable : 1;
	unsigned short	Int2ActHigh : 1;
	unsigned short	Int2Status : 1;
	unsigned short	PciIntEnable : 1;
	unsigned short	SoftIntrrupt : 1;
	unsigned short	Int1EdgeTrig : 1;
	unsigned short	Int2EdgeTrig : 1;
	unsigned short	Int1Clear : 1;
	unsigned short	Int2Clear : 1;
	unsigned short	Reserved : 4;
} P9030_INTCSR;

typedef union
_plx90_wordreg_
{
	P9030_INTCSR	intcsr;
	unsigned short	r;
} PLX90_WORDREG;

#define P9030_SetIntcsr(dwPort, wIntcsr) OP_outw(dwPort, LINTCSR, wIntcsr)

#endif



