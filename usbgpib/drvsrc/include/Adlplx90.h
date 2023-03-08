#ifndef		_ADLPLX90_H_
#define		_ADLPLX90_H_


void PLX90_ClearController(void __iomem *dwPort);
void PLX90_SoftReset(void __iomem *dwPort);
void PLX90_SetInt(void __iomem *dwPort, unsigned short wPin, unsigned short wEnable);
void PLX90_SetUserIoPin(void __iomem *dwPort,unsigned short wPin,unsigned short wDat);
void PLX90_GetUserIoPin(void __iomem *dwPort,unsigned short wPin,unsigned short* wpDat);
unsigned short PLX90_IsrCheckSource(void __iomem *dwPort); // 980918/pang
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
#define		ADL_OP_LAS0RR			0x00
#define		ADL_OP_LAS1RR			0x04
#define		ADL_OP_LAS2RR			0x08
#define		ADL_OP_LAS3RR			0x0C
#define		ADL_OP_EROMRR			0x10
#define		ADL_OP_LAS0BA			0x14
#define		ADL_OP_LAS1BA			0x18
#define		ADL_OP_LAS2BA			0x1C
#define		ADL_OP_LAS3BA			0x20
#define		ADL_OP_EROMBA			0x24
#define		ADL_OP_LAS0BRD			0x28
#define		ADL_OP_LAS1BRD			0x2C
#define		ADL_OP_LAS2BRD			0x30
#define		ADL_OP_LAS3BRD			0x34
#define		ADL_OP_EROMBRD			0x38
#define		ADL_OP_CS0BASE			0x3C
#define		ADL_OP_CS1BASE			0x40
#define		ADL_OP_CS2BASE			0x44
#define		ADL_OP_CS3BASE			0x48
#define		ADL_OP_LINTCSR			0x4C
#define		ADL_OP_CNTRL			0x50
#define		ADL_OP_EEPROM			0x53
/*--------------------------------------------------------------------------*/
#define		PLX_UIO_NO_USED			0x01
#define		PLX_UIO_INPUT			0x00
#define		PLX_UIO_OUT_H			0x06
#define		PLX_UIO_OUT_L			0x02


typedef	struct
{
	unsigned char	Int1Enable : 1;
	unsigned char	Int1ActHigh : 1;
	unsigned char	Int1Status : 1;
	unsigned char	Int2Enable : 1;
	unsigned char	Int2ActHigh : 1;
	unsigned char	Int2Status : 1;
	unsigned char	PciIntEnable : 1;
	unsigned char	SoftIntrrupt : 1;
} PLX_INTCSR;

typedef	struct
{
	unsigned char	ClockToggle : 1;
	unsigned char	ChipSelect : 1;
	unsigned char DataIn : 1;
	unsigned char	DataOut : 1;
	unsigned char	Valid : 1;
	unsigned char	ReloadConfig : 1;
	unsigned char	SoftReset : 1;
	unsigned char	MaskRevision : 1;
} PLX_EEPROM;

typedef union
_plx90_bytereg_
{
	PLX_EEPROM	eeprom;
	PLX_INTCSR	intcsr;
	unsigned char	r;
} PLX90_BYTEREG;

#define PLX90_SetIntcsr(dwPort, bIntcsr) OP_outb(dwPort, LINTCSR, bIntcsr)

#endif



