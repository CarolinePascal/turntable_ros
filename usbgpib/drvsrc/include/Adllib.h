#ifndef		_ADL_LIB_H_
#define		_ADL_LIB_H_

#include <asm/io.h>
//#include "/usr/src/linux-headers-4.4.38-tegra/arch/arm64/include/asm/io.h"
#include <linux/fs.h> // for 'struct fasync_struct'
//#include "../include/gpib_user.h"
#include "../include/Adldev.h"
/*---- defination for debug output ----*/
#ifdef _DEBUG
#define AdlDump(STRING)		DbgPrint STRING;
#else
#define	AdlDump(STRING)
#endif
#define DAS_BIT(Val)	Val ? 1 : 0

typedef union Das4Bytes {
	uint8_t	 c[4];
	uint16_t w[2];
	uint32_t d;
} DAS_4BYTE;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//---- Macro for OP port I/O ----
#define	OP_Port(pDevExt) (pDevExt->iobarmap[0])

#define	OP_outb(Port,Ofst,Value) iowrite8((U8)Value, (Port+ADL_OP_##Ofst))

#define	OP_outw(Port,Ofst,Value) iowrite16((U16)Value, (Port+ADL_OP_##Ofst))

#define	OP_outd(Port,Ofst,Value) iowrite32((U32)Value, (Port+ADL_OP_##Ofst))

#define	OP_inpb(Port,Ofst) ioread8((Port+ADL_OP_##Ofst))

#define	OP_inpw(Port,Ofst) ioread16((Port+ADL_OP_##Ofst))

#define	OP_inpd(Port,Ofst) ioread32((Port+ADL_OP_##Ofst))

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//---- Macro for PT port I/O ----
#define	PT_Port(pDevExt) (pDevExt->iobarmap[1])

#define	PT_outb(Port,Ofst,Value) iowrite8((U8)Value, ((Port+ADL_PT_##Ofst))

#define	PT_outw(Port,Ofst,Value) iowrite16((U16)Value, (Port+ADL_PT_##Ofst))

#define	PT_outd(Port,Ofst,Value) iowrite32((U32)Value, ((Port+ADL_PT_##Ofst))

#define	PT_inpb(Port,Ofst) ioread8(((Port+ADL_PT_##Ofst))

#define	PT_inpw(Port,Ofst) ioread16((Port+ADL_PT_##Ofst))

#define	PT_inpd(Port,Ofst) ioread32((Port+ADL_PT_##Ofst))
#if 0
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//---- Macro for P2 port I/O ----
#define	P2_Port(pDevExt) (pDevExt->ioaddr[2])

#define	P2_outb(Port,Ofst,Value) iowrite8((U8)Value, (Port+ADL_P2_##Ofst))

#define	P2_outw(Port,Ofst,Value) iowrite16((U16)Value, (Port+ADL_P2_##Ofst))

#define	P2_outd(Port,Ofst,Value) iowrite32((U32)Value, (Port+ADL_P2_##Ofst))

#define	P2_inpb(Port,Ofst) ioread8((Port+ADL_P2_##Ofst))

#define	P2_inpw(Port,Ofst) ioread16((Port+ADL_P2_##Ofst))

#define	P2_inpd(Port,Ofst) ioread32((Port+ADL_P2_##Ofst))
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//---- Macro for P3 port I/O ----
#define	P3_Port(pDevExt) (pDevExt->dwPortAddr[3])

#define	P3_outb(Port,Ofst,Value) outb_p((U8)Value, (U32)(Port+ADL_P3_##Ofst))

#define	P3_outw(Port,Ofst,Value) outw_p((U16)Value, (U32)(Port+ADL_P3_##Ofst))

#define	P3_outd(Port,Ofst,Value) outl_p((U32)Value, (U32)(Port+ADL_P3_##Ofst))

#define	P3_inpb(Port,Ofst) inb_p((U32)(Port+ADL_P3_##Ofst))

#define	P3_inpw(Port,Ofst) inw_p((U32)(Port+ADL_P3_##Ofst))

#define	P3_inpd(Port,Ofst) inl_p((U32)(Port+ADL_P3_##Ofst))
#endif
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//---- Macro for General port I/O ----
#define	outB(Port,Value) iowrite8((U8)(Value),(Port))

#define	outW(Port,Value) iowrite16((U16)(Value),(Port))

#define	outD(Port,Value) iowrite32((U32)(Value),(Port))

#define	inpB(Port) ioread8((Port))

#define	inpW(Port) ioread16((Port))

#define	inpD(Port) iorea32((Port))

/*---- Macro for Memory port I/O ----*/
#define	REG_outb(Port,Ofst,Value) \
(*(volatile unsigned char *)(Port+ADL_OP_##Ofst) = (Value))

#define	REG_outw(Port,Ofst,Value) \
(*(volatile unsigned short *)(Port+ADL_OP_##Ofst) = (Value))

#define	REG_outd(Port,Ofst,Value) \
(*(volatile unsigned int *)(Port+ADL_OP_##Ofst) = (Value))

#define	REG_inpb(Port,Ofst) \
(*(volatile unsigned char *)(Port+ADL_OP_##Ofst))

#define	REG_inpw(Port,Ofst) \
(*(volatile unsigned short *)(Port+ADL_OP_##Ofst))

#define	REG_inpd(Port,Ofst) \
(*(volatile unsigned int *)(Port+ADL_OP_##Ofst))

#define read_byte( priv, register_num) \
ioport_read_byte_with_lock(priv, register_num)

#define write_byte( priv, data, register_num) \
ioport_write_byte_with_lock(priv, data, register_num)

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*---- defination for IOCTL system buffer reference ----*/
#define		ADL_UCS_MAX		256
#define		ADL_UCS_NUM		16
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

#ifndef io_remap_page_range
#define io_remap_page_range(p1, p2, p3, p4, p5) io_remap_pfn_range(p1, p2, p3>>PAGE_SHIFT, p4, p5)
#endif

#ifndef SA_INTERRUPT
	#define SA_INTERRUPT            IRQF_DISABLED
#endif
#ifndef SA_SHIRQ
	#define SA_SHIRQ                IRQF_SHARED
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	#define class_device_create(a, b, c, d, e, f) device_create(a, b, c, d, e, f)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	#define class_device_destroy(a, b) device_destroy(a, b)
#endif

#endif


