/***************************************************************************
 util.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from nec7210/util.c of the Linux GPIB Package driver 
 		by Frank Mori Hess    
 Copyright (C) 2001, 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 
//#include "../include/Adlsys.h"
#include <asm/io.h>
#include <linux/delay.h>
#include "../include/Adldev.h"
#include "../include/Adllib.h"
#include "gpib_user.h"
#include "gpib_proto.h"
#include "u348a.h"
#include "ezusbsys.h"

void adgpib_eos_enable(PU348A_DEVEXT priv, U8 eos_byte, int compare_8_bits)
{
	write_byte(priv, eos_byte, EOSR);
	priv->auxa_bits |= HR_REOS;
	if(compare_8_bits)
		priv->auxa_bits |= HR_BIN;
	else
		priv->auxa_bits &= ~HR_BIN;
	write_byte(priv, priv->auxa_bits, AUXMR);
}

void adgpib_eos_disable(PU348A_DEVEXT priv)
{
	priv->auxa_bits &= ~HR_REOS;
	write_byte(priv, priv->auxa_bits, AUXMR);
}

int adgpib_parallel_poll(PU348A_DEVEXT priv, U8 *result)
{

	U8 retval[3];
	//spin_lock_irqsave( &priv->register_page_lock, flags );
	retval[0] = retval[1] = retval[2] = 0;			
	UDServiceFire(priv, VR_GPIB_CTRL_PP, 0, 0, 0, NULL, 0);			
	wdt_start( priv, priv->usec_timeout+1000000 );
	do {
		GetGPIBResponse(priv, 3, retval);
	} while((retval[0] != DONE) && (!test_bit( TIMO_NUM, &priv->status )));
	wdt_remove( priv );
	if(retval[0] != DONE) {
		printk("ppoll error %d %d\n", retval[0], retval[1]);
		return -4;
	}
	*result =  retval[2];
	return 0;
}

void adgpib_ibppc( PU348A_DEVEXT priv, unsigned int configuration )
{
	write_byte( priv, PPR | configuration , AUXMR );
}

void adgpib_ibist( PU348A_DEVEXT priv, int ist )
{
	if( ist )
		write_byte( priv, AUX_SPPF , AUXMR );
	else
		write_byte( priv, AUX_CPPF , AUXMR );
}

void adgpib_ibrsv(PU348A_DEVEXT priv, uint8_t status)
{
	write_byte(priv, status, SPMR);	
}

uint8_t adgpib_spoll_status_readback( PU348A_DEVEXT priv )
{
	return read_byte( priv, SPSR );
}

void adgpib_primary_address(PU348A_DEVEXT priv, unsigned int address)
{
	write_byte(priv, address & ADDRESS_MASK, ADR);
}

void adgpib_secondary_address(PU348A_DEVEXT priv, unsigned int address, int enable)
{	
	uint8_t rwbuf[128];
	uint8_t rwcnt = 0;
	
	memset(rwbuf, 0, 128);
	//yuan add 04/30/07 for multi register write
	rwcnt = 0;
	if(enable)
	{
		// put secondary address in address1
		rwbuf[rwcnt++] = ADR;
		rwbuf[rwcnt++] = HR_ARS | (address & ADDRESS_MASK);		
		//...write_byte(priv, HR_ARS | (address & ADDRESS_MASK), ADR);
		// go to address mode 2
		priv->reg_bits[ ADMR ] &= ~HR_ADM0;
		priv->reg_bits[ ADMR ] |= HR_ADM1;
	}else
	{
		// disable address1 register
		rwbuf[rwcnt++] = ADR;
		rwbuf[rwcnt++] = HR_ARS | HR_DT | HR_DL;			
		//....write_byte(priv, HR_ARS | HR_DT | HR_DL, ADR);
		// go to address mode 1
		priv->reg_bits[ ADMR ] |= HR_ADM0;
		priv->reg_bits[ ADMR ] &= ~HR_ADM1;
	}
	rwbuf[rwcnt++] = ADMR;
	rwbuf[rwcnt++] = priv->reg_bits[ ADMR ];
	ioport_Mult_Read_write(priv, 0, 0, rwcnt, rwbuf); 		
}

unsigned int _update_status( PU348A_DEVEXT priv )
{
	int address_status_bits;
	unsigned long flags;

	if(priv == NULL) return 0;

	address_status_bits = read_byte(priv, ADSR);
	spin_lock_irqsave( &priv->spinlock, flags );
	if(address_status_bits & HR_CIC)
		set_bit(CIC_NUM, &priv->status);
	else
		clear_bit(CIC_NUM, &priv->status);
	// check for talker/listener addressed
	if(address_status_bits & HR_TA)
	{
		set_bit(TACS_NUM, &priv->status);
	}else
		clear_bit(TACS_NUM, &priv->status);
	if(address_status_bits & HR_LA)
	{
		set_bit(LACS_NUM, &priv->status);
	}else
		clear_bit(LACS_NUM, &priv->status);
	if(address_status_bits & HR_NATN)
	{
		clear_bit(ATN_NUM, &priv->status);
	}else
	{
		set_bit(ATN_NUM, &priv->status);
	}
	spin_unlock_irqrestore( &priv->spinlock, flags );
	return priv->status;
}

unsigned int adgpib_update_status(PU348A_DEVEXT board, unsigned int clear_mask )
{
	unsigned long flags;
	unsigned int retval;

	//spin_lock_irqsave( &board->spinlock, flags );
	retval = _update_status( board );
	spin_lock_irqsave( &board->spinlock, flags);
	board->status &= ~clear_mask;
	spin_unlock_irqrestore( &board->spinlock, flags );
    	return retval;
}

unsigned int adgpib_set_reg_bits( PU348A_DEVEXT priv, unsigned int reg,
	unsigned int mask, unsigned int bits )
{
	priv->reg_bits[ reg ] &= ~mask;
	priv->reg_bits[ reg ] |= mask & bits;
	write_byte( priv, priv->reg_bits[ reg ], reg );
	return priv->reg_bits[ reg ];
}

void adgpib_set_handshake_mode( PU348A_DEVEXT priv, int mode )
{
	unsigned long flags;

	spin_lock_irqsave( &priv->spinlock, flags );
	priv->auxa_bits &= ~HR_HANDSHAKE_MASK;
	priv->auxa_bits |= ( mode & HR_HANDSHAKE_MASK );

	spin_unlock_irqrestore( &priv->spinlock, flags );
	write_byte( priv, priv->auxa_bits, AUXMR );
}

uint8_t adgpib_read_data ( PU348A_DEVEXT priv, int *end )
{
	unsigned long flags;
	uint8_t data;

	spin_lock_irqsave( &priv->spinlock, flags );
	clear_bit( READ_READY_BN, &priv->state );

	spin_unlock_irqrestore( &priv->spinlock, flags );
	data = read_byte( priv, DIR );
	if( test_and_clear_bit_sync( priv, RECEIVED_END_BN, &priv->state ) )
		*end = 1;
	else
		*end = 0;
	return data;
}

int adgpib_ibcac(PU348A_DEVEXT board, int syncronous)
{
	int i;
	const int timeout = 100000;
	unsigned int adsr_bits = 0;	
	uint8_t retval[2];

	retval[0] = retval[1] = 0;			
	UDServiceFire(board, VR_GPIB_CAC, (U8) (syncronous&0x01), 0, 0, NULL, 0);	
	GetGPIBResponse(board, 2, retval);
	if(retval[0] != DONE) {
		UDServiceFire(board, VR_GPIB_CAC, (U8) (syncronous&0x01), 0, 0, NULL, 0);
		GetGPIBResponse(board, 2, retval);
	}

	if((retval[0] != DONE) || (retval[1] != 0))
		return -4;
	clear_bit( WRITE_READY_BN, &board->state );
	
	return 0;
}

void adgpib_ibsre(PU348A_DEVEXT priv, int enable)
{
	if(enable)
		write_byte(priv, AUX_SREN, AUXMR);
	else
		write_byte(priv, AUX_CREN, AUXMR);
}

int adgpib_ibgts (PU348A_DEVEXT priv)
{
	int i;
	U8 retval[2];

	retval[0] = retval[1] = 0;			
	UDServiceFire(priv, VR_GPIB_GTS, 0, 0, 0, NULL, 0);	
	GetGPIBResponse(priv, 2, retval);
	if(retval[0] != DONE) {
		UDServiceFire(priv, VR_GPIB_GTS, 0, 0, 0, NULL, 0);
		GetGPIBResponse(priv, 2, retval);
	}
	if((retval[0] != DONE) || (retval[1] != 1))
		return -4;
	return 0;
}

void adgpib_isr1_status( PU348A_DEVEXT priv, int status1 )
{
	
	if( status1 & HR_DEC )
	{
		set_bit(DCAS_NUM, &priv->status);
		//retval[0] |= HR_DEC;
	}	

	if( status1 & HR_DET )
	{
		set_bit(DTAS_NUM, &priv->status);
		priv->status |= DTAS;
		//retval[0] |= HR_DET;
	} 
	
	return;
}

void adgpib_isr2_status( PU348A_DEVEXT priv, int status2 )
{
	if(status2 & HR_LOK)
		set_bit(LOK_NUM, &priv->status);
	else
		clear_bit(LOK_NUM, &priv->status);
	// change in remote status
	if(status2 & HR_REM)
		set_bit(REM_NUM, &priv->status);
	else
		clear_bit(REM_NUM, &priv->status);
	return;
}

void bit_control(bit_control_t *bc)
{							
    *(bc->val) = (bc->mask & (*(bc->addr))) != 0;
     switch(bc->op) {
	  case 0: //test_clear
		  *(bc->addr) &= ~bc->mask;
		  break;
	  case 1: //set
          *(bc->addr) |= bc->mask;
		  break;
	 }
}	
