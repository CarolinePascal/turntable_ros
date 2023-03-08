/***************************************************************************
 p348a.h
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from include/nec7210.h and cb7210/cb7210.h of 
 							the Linux GPIB Package driver by Frank Mori Hess      
 Copyright (C) 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/
 
#ifndef U348A_H
#define U348A_H

#include <linux/interrupt.h>

#include "../include/Adldev.h"
#include "../include/Adllib.h"

#define getBit(reg_var, bit)   ((reg_var >> bit) & 1L)
#define setBit(reg_var, bit)   { reg_var = (reg_var | (1L << bit));  }
#define resetBit(reg_var, bit) { reg_var = (reg_var & ~(1L << bit)); }

enum
{
	PIO_IN_PROGRESS_BN,	// pio transfer in progress
	DMA_READ_IN_PROGRESS_BN,	// dma read transfer in progress
	DMA_WRITE_IN_PROGRESS_BN,	// dma write transfer in progress
	READ_READY_BN,	// board has data byte available to read
	WRITE_READY_BN,	// board is ready to send a data byte
	COMMAND_READY_BN,	// board is ready to send a command byte
	RECEIVED_END_BN,	// received END
	BUS_ERROR_BN,	// output error has occurred
	RFD_HOLDOFF_BN,	// rfd holdoff in effect
	DEV_CLEAR_BN,	// device clear received
};

size_t adgpib_read(PU348A_DEVEXT priv, gpib_descriptor_t * desc, uint8_t *buffer, size_t length, int *end);
size_t adgpib_write(PU348A_DEVEXT priv, gpib_descriptor_t * desc, uint8_t *buffer, size_t length, int send_eoi);
int adgpib_command(PU348A_DEVEXT priv, gpib_descriptor_t * desc, uint8_t *buffer, size_t length);
int adgpib_ibcac(PU348A_DEVEXT priv, int syncronous);
int adgpib_ibgts(PU348A_DEVEXT priv);
void adgpib_ibsre(PU348A_DEVEXT priv, int enable);
void adgpib_eos_enable(PU348A_DEVEXT priv, uint8_t eos_bytes, int compare_8_bits);
void adgpib_eos_disable(PU348A_DEVEXT priv);
unsigned int adgpib_update_status( PU348A_DEVEXT priv, unsigned int clear_mask );
void adgpib_primary_address( const PU348A_DEVEXT priv, unsigned int address);
void adgpib_secondary_address( const PU348A_DEVEXT priv, unsigned int address, int enable);
int adgpib_parallel_poll(PU348A_DEVEXT priv, uint8_t *result);
void adgpib_ibrsv(PU348A_DEVEXT priv, uint8_t status);
void adgpib_ibppc( PU348A_DEVEXT priv, unsigned int configuration );
void adgpib_ibist( PU348A_DEVEXT priv, int ist );
uint8_t adgpib_spoll_status_readback( PU348A_DEVEXT priv );
unsigned int adgpib_t1_delay( PU348A_DEVEXT priv, unsigned int nano_sec );

void u348a_board_reset( PU348A_DEVEXT priv );
void u348a_board_online( PU348A_DEVEXT priv );
unsigned int adgpib_set_reg_bits( PU348A_DEVEXT priv, unsigned int reg, unsigned int mask, unsigned int bits );
void adgpib_set_handshake_mode( PU348A_DEVEXT priv, int mode );
void adgpib_release_rfd_holdoff( PU348A_DEVEXT priv );
uint8_t adgpib_read_data( PU348A_DEVEXT priv, int *end );

uint8_t ioport_read_byte(PU348A_DEVEXT priv, unsigned int register_num);
void ioport_write_byte(PU348A_DEVEXT priv, uint8_t data, unsigned int register_num);
uint8_t ioport_read_byte_with_lock(PU348A_DEVEXT priv, unsigned int register_num);
void ioport_write_byte_with_lock(PU348A_DEVEXT priv, uint8_t data, unsigned int register_num);
int ioport_Mult_Read_write(U348A_DEVEXT *priv, uint8_t dir, uint8_t index, uint8_t length, uint8_t* buffer);
void adgpib_isr1_status( PU348A_DEVEXT priv, int status1 );
void adgpib_isr2_status( PU348A_DEVEXT priv, int status2 );

// p348a has 8 registers
static const int p348a_num_registers = 8;

/* p348a register numbers (might need to be multiplied by
 * a board-dependent offset to get actually io address offset)
 */
// write registers
enum p348a_write_regs
{
	CDOR,	// command/data out
	IMR1,	// interrupt mask 1
	IMR2,	// interrupt mask 2
	SPMR,	// serial poll mode
	ADMR,	// address mode
	AUXMR,	// auxilliary mode
	ADR,	// address
	EOSR,	// end-of-string
};
// read registers
enum p348a_read_regs
{
	DIR,	// data in
	ISR1,	// interrupt status 1
	ISR2,	// interrupt status 2
	SPSR,	// serial poll status
	ADSR,	// address status
	CPTR,	// command pass though
	ADR0,	// address 1
	ADR1,	// address 2
};

//bit definitions common to nec-7210 compatible registers
enum isr0_bits
{
	HR_SYNC = ( 1 << 0 ),	
	HR_ATNI = ( 1 << 2 ),
	HR_IFCI = ( 1 << 3 ),
	HR_EOS = ( 1 << 4 ),	
	HR_NL = ( 1 << 5 ),
	HR_STBO = ( 1 << 6 ),
	HR_nba = ( 1 << 7 ),
};

// IMR0: interrupt mask register 0
enum imr0_bits
{
	HR_SYNCIE = ( 1 << 0 ),	
	HR_ATNIIE = ( 1 << 2 ),
	HR_IFCIIE = ( 1 << 3 ),
	HR_NLIE = ( 1 << 5 ),
	HR_STBOIE = ( 1 << 6 ),
	HR_GLINT = ( 1 << 7 ),
};
// ISR1: interrupt status register 1
enum isr1_bits
{
	HR_DI = ( 1 << 0 ),
	HR_DO = ( 1 << 1 ),
	HR_ERR = ( 1 << 2 ),
	HR_DEC = ( 1 << 3 ),
	HR_END = ( 1 << 4 ),
	HR_DET = ( 1 << 5 ),
	HR_APT = ( 1 << 6 ),
	HR_CPT = ( 1 << 7 ),
};

// IMR1: interrupt mask register 1
enum imr1_bits
{
	HR_DIIE = ( 1 << 0 ),
	HR_DOIE = ( 1 << 1 ),
	HR_ERRIE = ( 1 << 2 ),
	HR_DECIE = ( 1 << 3 ),
	HR_ENDIE = ( 1 << 4 ),
	HR_DETIE = ( 1 << 5 ),
	HR_APTIE = ( 1 << 6 ),
	HR_CPTIE = ( 1 << 7 ),
};

// ISR2, interrupt status register 2
enum isr2_bits
{
	HR_ADSC = ( 1 << 0 ),
	HR_REMC = ( 1 << 1 ),
	HR_LOKC = ( 1 << 2 ),
	HR_CO = ( 1 << 3 ),
	HR_REM = ( 1 << 4 ),
	HR_LOK = ( 1 << 5 ),
	HR_SRQI = ( 1 << 6 ),
	HR_INT = ( 1 << 7 ),
};

// IMR2, interrupt mask register 2
enum imr2_bits
{
	// all the bits in this register that enable interrupts
	IMR2_ENABLE_INTR_MASK = 0x4f,
	HR_ACIE = ( 1 << 0 ),
	HR_REMIE = ( 1 << 1 ),
	HR_LOKIE = ( 1 << 2 ),
	HR_COIE = ( 1 << 3 ),
	HR_DMAI = ( 1 << 4 ),
	HR_DMAO = ( 1 << 5 ),
	HR_SRQIE = ( 1 << 6 ),
};

// SPSR, serial poll status register
enum spsr_bits
{
	HR_PEND = ( 1 << 6 ),
};

// SPMR, serial poll mode register
enum spmr_bits
{
	HR_RSV = ( 1 << 6 ),
};

// ADSR, address status register
enum adsr_bits
{
	HR_MJMN = ( 1 << 0 ),
	HR_TA = ( 1 << 1 ),
	HR_LA = ( 1 << 2 ),
	HR_TPAS = ( 1 << 3 ),
	HR_LPAS = ( 1 << 4 ),
	HR_SPMS = ( 1 << 5 ), 
	HR_NATN = ( 1 << 6 ),
	HR_CIC = ( 1 << 7 ),
};

// ADMR
enum admr_bits
{
	HR_ADM0 = ( 1 << 0 ),
	HR_ADM1 = ( 1 << 1 ),
	HR_TRM0 = ( 1 << 4 ),
	HR_TRM1 = ( 1 << 5 ),
	HR_TRM_EOIOE_TRIG = 0,
	HR_TRM_CIC_TRIG = HR_TRM0,
	HR_TRM_CIC_EOIOE = HR_TRM1,
	HR_TRM_CIC_PE = HR_TRM0 | HR_TRM1,
	HR_LON = ( 1 << 6 ),
	HR_TON = ( 1 << 7 ),
};

// ADR
enum adr_bits
{
	ADDRESS_MASK = 0x1f,
	HR_DL = ( 1 << 5 ),
	HR_DT = ( 1 << 6 ),
	HR_ARS = ( 1 << 7 ),
};

// ADR1
enum adr1_bits
{
	HR_EOI = ( 1 << 7 ),
};

// AUXMR
enum auxmr_bits
{
	ICR = 0x20,
	PPR = 0x60,
	AUXRA = 0x80,
	AUXRB = 0xa0,
	AUXRE = 0xc0,
	AUXRF = 0xD0,
	AUXRG = 0x40,
	AUXRI = 0xE0,
};

// auxiliary register A
enum auxra_bits
{
	HR_HANDSHAKE_MASK = 0x3,
	HR_HLDA = 0x1,
	HR_HLDE = 0x2,
	HR_LCM = 0x3,	/* auxra listen continuous */
	HR_REOS = 0x4,
	HR_XEOS = 0x8,
	HR_BIN = 0x10,
};

//auxiliary register B
enum auxrb_bits
{
	HR_CPTE = ( 1 << 0 ),
	HR_SPEOI = ( 1 << 1 ),
	HR_TRI = ( 1 << 2 ),
	HR_INV = ( 1 << 3 ),
	HR_ISS = ( 1 << 4 ),
};

enum auxre_bits
{
	HR_DAC_HLD_DCAS = 0x1,	/* perform DAC holdoff on receiving clear */
	HR_DAC_HLD_DTAS = 0x2,	/* perform DAC holdoff on receiving trigger */
};
//  auxiliary register G
enum auxrg_bits
{
	HR_CHES = ( 1 << 0 ),
	HR_DISTCT = ( 1 << 1 ),
	HR_RPP2 = ( 1 << 2 ),
	HR_NTNL = ( 1 << 3 ),	
};

// auxiliary register i
enum auxri_bits
{
	HR_SISB = ( 1 << 0 ),	
	HR_PP2 = ( 1 << 2 ),
	HR_USTD = ( 1 << 3 ),	
};
// parallel poll register
enum ppr_bits
{
	HR_PPS = ( 1 << 3 ),
	HR_PPU = ( 1 << 4 ),
};

/* Auxiliary Commands */
enum aux_cmds
{
	AUX_PON = 0x0,	/* Immediate Execute pon                  */
	AUX_CPPF = 0x1,	/* Clear Parallel Poll Flag               */
	AUX_CR = 0x2,	/* Chip Reset                             */
	AUX_FH = 0x3,	/* Finish Handshake                       */
	AUX_TRIG = 0x4,	/* Trigger                                */
	AUX_RTL = 0x5,	/* Return to local                        */
	AUX_SEOI = 0x6,	/* Send EOI                               */
	AUX_NVAL = 0x7,	/* Non-Valid Secondary Command or Address */
	AUX_SPPF = 0x9,	/* Set Parallel Poll Flag                 */
	AUX_VAL = 0xf,	/* Valid Secondary Command or Address     */
	AUX_GTS = 0x10,	/* Go To Standby                          */
	AUX_TCA = 0x11,	/* Take Control Asynchronously            */
	AUX_TCS = 0x12,	/* Take Control Synchronously             */
	AUX_LTN = 0x13,	/* Listen                                 */
	AUX_DSC = 0x14,	/* Disable System Control                 */
	AUX_CIFC = 0x16,	/* Clear IFC                              */
	AUX_CREN = 0x17,	/* Clear REN                              */
	AUX_REQT = 0x18,	/* rsv TRUE                              */
	AUX_REQF = 0x19,	/* rsv FALSE                             */
	AUX_TCSE = 0x1a,	/* Take Control Synchronously on End      */
	AUX_LTNC = 0x1b,	/* Listen in Continuous Mode              */
	AUX_LUN = 0x1c,	/* Local Unlisten                         */
	AUX_EPP = 0x1d,	/* Execute Parallel Poll                  */
	AUX_SIFC = 0x1e,	/* Set IFC                                */
	AUX_SREN = 0x1f,	/* Set REN                                */
	AUX_PAGEIN = 0x50,  /* page in register */	
	AUX_HLDI = 0x51,  /* holdoff handshake immediately */	
	AUX_SCO = 0x52,		/* enable SC                             */
	AUX_SCI = 0x53,		/* Non SC                               */
	AUX_CDET = 0x54,		/* clear DET                   */
	AUX_CEND = 0x55,		/* clear END                   */
	AUX_CDEC = 0x56,		/* clear END                   */
	AUX_CERR = 0x57,		/* clear END                   */
	AUX_CSRQ = 0x58,		/* clear END                   */
	AUX_CLOKC = 0x59,		/* clear END                   */
	AUX_CREMC = 0x5A,		/* clear END                   */
	AUX_CADSC = 0x5B,		/* clear END                   */
	AUX_CIFCI = 0x5C,		/* clear END                   */
	AUX_CATNI = 0x5D,		/* clear END                   */
	AUX_CSYNC = 0x5E,		/* clear SYNC                   */
	AUX_SSYNC = 0x5F,		/* Set SYNC                     */
};

// fifo size in bytes
static const int cb7210_fifo_size = 1024;
static const int ad9607_fifo_size = 2048;

// cb7210 specific registers and bits
enum p348a_regs
{
	BUS_STATUS = 0x7,
	IMR0 = 0x6,
	ISR0 = 0x6
};

#define CHIP9607_PAGE 0
#define BUS_STATUS_PAGE 1

/*static inline int cb7210_page_in_bits( unsigned int page )
{
	return 0x50 | (page & 0xf);
}*/

enum fifo_regs
{
	//write registers
	OFIFO = 0x8,	// out fifo
	//read registers
	IFIFO = 0x8,	// in fifo
};

enum bus_status_bits
{
	BSR_ATN_BIT_9607 = 0x80,
	BSR_EOI_BIT_9607 = 0x08,
	BSR_SRQ_BIT_9607 = 0x4,
	BSR_IFC_BIT_9607 = 0x2,
	BSR_REN_BIT_9607 = 0x1,
	BSR_DAV_BIT_9607 = 0x40,
	BSR_NRFD_BIT_9607 = 0x10,
	BSR_NDAC_BIT_9607 = 0x20,
};

enum hs_mode_bits
{
	HS_ENABLE_MASK = 0xc0,
	HS_TX_ENABLE = ( 1 << 6 ),
	HS_RX_ENABLE = ( 1 << 7 ),
	HS_FIFO_CLR = ( 1 << 0 ),
	HS_RX_DIR = ( 1 << 1 ),
	HS_RESET7210 = ( 1 << 2 ),
	HS_SYS_CONTROL = ( 1 << 3 ),
	//HS_EOI_EN = ( 1 << 3 ),
	HS_THRES_INT_EN = ( 1 << 4 ),
	HS_TX_DONE_INT_EN = ( 1 << 5 ),
	//HS_CLR_SRQ_INT = ( 1 << 5 ),
	HS_CLR_THRES_INT = ( 1 << 6 ),
	HS_CLR_TX_DONE_INT = ( 1 << 7 ),
};

enum hs_regs_9607
{
	//write registers
	RX_FIFO_THRESHOLD = 0x0c, /* FIFO threshold register */
	TX_FIFO_THRESHOLD = 0x0e, /* FIFO threshold register */
	RX_TRANSFER_COUNTER	= 0x10,  //r/w
        FIFO_CMD = 0x14,	//wo
	INT_CLR = 0x15,	//w
	INT_CTL = 0x16,	//w
	INT_EN = 0x17,	//r/w
	//read registers
	RX_FIFO_COUNT = 0x0c, /* FIFO current count register */
	TX_FIFO_COUNT = 0x0e, /* FIFO current count register */
	FIFO_STATUS = 0x14,	/* HS_STATUS register */
	INT_STATUS = 0x15, /* RX_DATA_STATUS */
};


enum fifo_cmd_9607
{
	RX_FIFO_Stop = 0,
	TX_FIFO_Stop = 1,
	RX_FIFO_Pause = 2,
	TX_FIFO_Pause = 3,
	RX_FIFO_Resume = 4,
	TX_FIFO_Resume = 5,
        RX_FIFO_RunCont = 0x6,
	TX_FIFO_RunCont = 0x7,
	RX_FIFO_RunWithTranCount = 0x8,
	RX_FIFO_CLR_PausedOnEnd = 0xa,
	RX_FIFO_SET_PausedOnEnd = 0xb,
	TX_FIFO_InvalidEnd = 0x10,
	TX_FIFO_SetEnd1 = 0x11,
	TX_FIFO_SetEnd2 = 0x12,
	TX_FIFO_SetEnd3 = 0x13,
	TX_FIFO_SetEnd4 = 0x14,
	TX_FIFO_InvalidSEOI = 0x20,
	TX_FIFO_SEOI1 = 0x21,
	TX_FIFO_SEOI2 = 0x22,
	TX_FIFO_SEOI3 = 0x23,
	TX_FIFO_SEOI4 = 0x24
};
enum fifo_status_9607
{
	RX_FIFO_Stopped = 0,
	RX_FIFO_Paused_Cont = 2,
	RX_FIFO_Paused_TC = 3,
	RX_FIFO_Running_Cont = 6,
	RX_FIFO_Running_TC = 7,
	RX_FIFO_Paused_End = 0x8,

	TX_FIFO_Stopped = 0x0,
	TX_FIFO_Paused = 0x10,
	TX_FIFO_Running_Cont = 0x20,
	TX_FIFO_ENDVAL = 0x40,
	TX_FIFO_SEOIVAL = 0x80,
};

enum int_ctl_bits_9607
{
	RX_FULL_INT_EN = (1 << 0),
	RX_THRES_INT_EN = ( 1 << 1 ),
	RX_COUNT_OVER_EN = ( 1 << 2 ),
	RX_END_INT_EN = ( 1 << 3 ),
	TX_EMPTY_INT_EN = (1 << 4),
	TX_THRES_INT_EN = ( 1 << 5 ),
	TX_DONE_INT_EN = ( 1 << 6 ),
};

enum int_sts_bits_9607
{
	RX_FULL = (1 << 0),
	RX_THRES_INT = ( 1 << 1 ),
	RX_COUNT_OVER = ( 1 << 2 ),
	RX_END_INT = ( 1 << 3 ),
	TX_EMPTY = (1 << 4),
	TX_THRES_INT = ( 1 << 5 ),
	TX_DONE_INT = ( 1 << 6 ),
};

static inline int page_in_bits( unsigned int page )
{
	return 0x50 | (page & 0xf);
}

static __inline int page_in_register( unsigned int page,  unsigned int register_num)
{
	return (((page&0x1)<<8)|(register_num&0xff));
}

static __inline uint8_t paged_read_byte( PU348A_DEVEXT priv,
	unsigned int register_num, unsigned int page )
{
	uint8_t retval;
	retval = ioport_read_byte_with_lock(priv, page_in_register(page, register_num));	
	return retval;
}	
	
#endif

