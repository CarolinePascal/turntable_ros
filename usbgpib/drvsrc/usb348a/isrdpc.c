/***************************************************************************
 isrdpc.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from nec7210/interrupt.c of the Linux GPIB Package driver
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
 
#include <linux/sched.h>
#include <linux/interrupt.h>
#include "../include/Adldev.h"
#include "../include/Adllib.h"
#include "../include/Adlp9030.h"
#include "p348a.h"

/*----------------------------------------------------------------------------------*/

int ReadData(PPCI_DEVEXT pDevExt);

int handle_ad7210_interrupt( PPCI_DEVEXT priv, int status0, int status1, int status2 )
{
	//devices request service interrupt
	if(status2 & HR_SRQI)
	{
		set_bit(SRQI_NUM, &priv->status);
		if(priv->auxi_bits & HR_SISB)
			outB( PT_Port(priv)+ AUXMR , AUX_CSRQ);
	} 

	// lockout status change
	if(status2 & HR_LOKC)
	{
		if(priv->auxi_bits & HR_SISB)
			outB( PT_Port(priv)+ AUXMR , AUX_CLOKC);
	}
	// lockout status
	if(status2 & HR_LOK)
		set_bit(LOK_NUM, &priv->status);
	else
		clear_bit(LOK_NUM, &priv->status);
	// remote status change
	if(status2 & HR_REMC)
	{
	if(priv->auxi_bits & HR_SISB)
	  outB( PT_Port(priv)+ AUXMR , AUX_CREMC);	
	//printk("HR_REM %x\n", status2);
	}
	//remote status
	if(status2 & HR_REM)
		set_bit(REM_NUM, &priv->status);
	else
		clear_bit(REM_NUM, &priv->status);
	 
	if(!(priv->auxi_bits & HR_SISB)) {

	 if((status1 & HR_DI))
	 {
		set_bit(READ_READY_BN, &priv->state);
		if( ( priv->auxa_bits & HR_HANDSHAKE_MASK ) == HR_HLDA )
			set_bit( RFD_HOLDOFF_BN, &priv->state);
	 }

	 // get END status
	 if(status1 & HR_END)
	 {
		set_bit(RECEIVED_END_BN, &priv->state);
		if( ( priv->auxa_bits & HR_HANDSHAKE_MASK ) == HR_HLDE )
			set_bit( RFD_HOLDOFF_BN, &priv->state);
	 }

	 if(status1 & HR_DO) 
		set_bit(WRITE_READY_BN, &priv->state);	
	// command can be sent
	 if(status2 & HR_CO)
	 {
		set_bit(COMMAND_READY_BN, &priv->state);
	 }
	}
	// command pass through status
	if(status1 & HR_CPT)
	{
		unsigned char command;

	    command = inpB( PT_Port(priv)+ CPTR );
	    outB( PT_Port(priv)+ AUXMR , AUX_NVAL);
		//printk("HR_CPT\n");
	}
	// err
	if(status1 & HR_ERR)
	{
		set_bit( BUS_ERROR_BN, &priv->state );
		if(priv->auxi_bits & HR_SISB)
			outB( PT_Port(priv)+ AUXMR , AUX_CERR);
		//printk("HR_ERR\n");
	}
	// device clear cmd
	if( status1 & HR_DEC )
	{
		//push_gpib_event( board, EventDevClr );
		set_bit( DEV_CLEAR_BN, &priv->state );
		set_bit(DCAS_NUM, &priv->status);
		if(priv->auxi_bits & HR_SISB)	
			outB( PT_Port(priv)+ AUXMR , AUX_CDEC);
		//printk("HR_DEC\n");
	}	
	// trigger cmd
	if( status1 & HR_DET )
	{
		set_bit(DTAS_NUM, &priv->status);
		priv->status |= DTAS;
		if(priv->auxi_bits & HR_SISB)
			outB( PT_Port(priv)+ AUXMR , AUX_CDET);
		//printk("HR_DET\n");
		//push_gpib_event( board, EventDevTrg );
	} 
	// interface clear interrupt
	if(status0 & HR_IFCI)
	{
		if(priv->auxi_bits & HR_SISB)
		outB( PT_Port(priv)+ AUXMR , AUX_CIFCI);
		//printk("int HR_IFCI\n");
	}
	//ATN asserted interrupt
	if(status0 & HR_ATNI)
	{
		if(priv->auxi_bits & HR_SISB)
		outB( PT_Port(priv)+ AUXMR , AUX_CATNI);
		//printk("int HR_ATNI\n");
	}
	// TA/LA change
	if(status2 & HR_ADSC)
	{
		unsigned char adsr_status = inpB( PT_Port(priv)+ 4 );
		//if( adsr_status & TACS)
		//	return 2;
		//if( adsr_status & LACS)
		//	return 3;		
		if(priv->auxi_bits & HR_SISB)
			outB( PT_Port(priv)+ AUXMR , AUX_CADSC);	
		//yuan add 11/23/09
		if(!(adsr_status & (HR_TA|HR_LA))) {
			adsr_status = inpB( PT_Port(priv)+ 4 );
			if(adsr_status & (HR_TA|HR_LA))
				 outB( PT_Port(priv)+ AUXMR , AUX_CADSC);
		}		
		if( adsr_status & HR_TA)
			return 2;
		if( adsr_status & HR_LA)
			return 3;	
		//printk("int HR_ADSC %x\n", adsr_status);	
	}
	return TRUE; //IRQ_HANDLED;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
irqreturn_t adgpib_interrupt_handler(int irq, void * dev_id, struct pt_regs *regs)
#else
irqreturn_t adgpib_interrupt_handler(int irq, void * dev_id)
#endif
{
  PPCI_DEVEXT  pDevExt;
  U16  wRetCode;
  int status1=0, status2=0, hs_status=0, res=0, e=0, status0=0;
  /*-----------------------------------------------------*/
  pDevExt = (PCI_DEVEXT *)dev_id;
  wRetCode = PLX90_IsrCheckSource(OP_Port(pDevExt));
  if ((! (wRetCode & 0xff00))||(!(wRetCode & 0x0003)))
      return IRQ_NONE;
  //printk("ISR %x\n", wRetCode);    
  if(wRetCode & 0x01) 
  {//int1
  unsigned long flags;
  	  spin_lock_irqsave( &pDevExt->register_page_lock, flags );
	  outB(PT_Port(pDevExt) + AUXMR, AUX_PAGEIN);
	  status0 = inpB(PT_Port(pDevExt) + ISR0);			  
	  spin_unlock_irqrestore( &pDevExt->register_page_lock, flags );
	  status1 = inpB( PT_Port(pDevExt)+ ISR1 );
	  status2 = inpB( PT_Port(pDevExt)+ ISR2 );
	  //printk("ISR %x %x %x\n", status1, status2, status0);    
 	  if(status1 & HR_END) 
		pDevExt->dwMemAddr[5]|=0x10;
	  res = handle_ad7210_interrupt( pDevExt, status0, status1, status2 );
  }
 if(wRetCode & 0x02) 
  {
	unsigned char intctl = inpB(PT_Port(pDevExt)+INT_CTL); 
	hs_status = inpB( PT_Port(pDevExt)+ INT_STATUS );
	if(!pDevExt->openable) {
			outB(PT_Port(pDevExt)+INT_CLR, hs_status);
			return IRQ_HANDLED;
	}
	
	pDevExt->page_reg_bits[7] = hs_status;
	if( (intctl & RX_THRES_INT_EN) && (hs_status & RX_THRES_INT) )
	{
		//if( pDevExt->hs_mode_bits & HS_RX_DIR ) 
		{			
			unsigned long remains=0;
			unsigned short thres=0;
			short int_en=0;

			//printk("RX HF int\n");
			remains = ReadData(pDevExt);
			//pDevExt->hs_mode_bits &= ~HS_THRES_INT_EN;
		    //outB(PT_Port(pDevExt)+HS_MODE, (pDevExt->hs_mode_bits&0x2f));
			outB(PT_Port(pDevExt)+INT_CTL, (intctl&(~RX_THRES_INT_EN)));//RX_END_INT_EN|RX_COUNT_OVER_EN);
			if(remains>=(ad9607_fifo_size/2)) {
				thres = ad9607_fifo_size / 2;
				int_en = 1;
			} ////else 
				////thres = remains;
			if(thres && (pDevExt->ThresHoldCnt!=thres)) {
				pDevExt->ThresHoldCnt = thres;
				outW(PT_Port(pDevExt)+RX_FIFO_THRESHOLD, pDevExt->ThresHoldCnt);
			}
			//yuan add 04/12/06
			/////if(pDevExt->state & (1<<RECEIVED_END_BN))
				/////e=1;
		    if(int_en)
			//pDevExt->hs_mode_bits &= ~HS_THRES_INT_EN;
			outB(PT_Port(pDevExt)+INT_CTL, intctl);
		    else
			    e= 0xff;
			//outB(PT_Port(pDevExt)+HS_MODE, (pDevExt->hs_mode_bits&0x3f)|HS_CLR_THRES_INT);
		}
		//yuan add 02/15/08
		if(pDevExt->state & (1<<RECEIVED_END_BN))
				e=1;

	}
	if( (intctl & TX_THRES_INT_EN) && (hs_status & TX_THRES_INT) )	
	{
			unsigned long remain=0, num_bytes=0;
			unsigned int *tmpB;
			unsigned short thres=0;

//		printk("TX HF int\n");
		outB(PT_Port(pDevExt)+INT_CTL, intctl&(~TX_THRES_INT_EN));
		remain = pDevExt->dwDevRB - pDevExt->dwDevRA;
		  if(remain>1) {
			if( hs_status & TX_EMPTY )
				num_bytes = ad9607_fifo_size;
			else num_bytes = ad9607_fifo_size - pDevExt->ThresHoldCnt;
			if( num_bytes >= remain ) {
				remain -= 1;
				num_bytes = remain;
			}
			remain=num_bytes/4;
		  tmpB = (unsigned int *) &(pDevExt->buffer[pDevExt->dwDevRA]);
			outsl( PT_Port(pDevExt)+OFIFO, tmpB, remain );
			//WRITE_PORT_BUFFER_ULONG (PT_Port(pDevExt)+OFIFO, tmpB, remain);
			pDevExt->dwDevRA += remain*4;
			remain=num_bytes%4;
			tmpB = (unsigned int *) &(pDevExt->buffer[pDevExt->dwDevRA]);	
			outsb( PT_Port(pDevExt)+OFIFO, (unsigned char *) tmpB, remain );		
			pDevExt->dwDevRA += remain;
			
			remain = pDevExt->dwDevRB - pDevExt->dwDevRA;
			if(remain>=(ad9607_fifo_size/2))
				thres = ad9607_fifo_size / 2;
			else 
				thres = ad9607_fifo_size-remain;
			if(thres && (pDevExt->ThresHoldCnt!=thres)) {
				pDevExt->ThresHoldCnt = thres;
				outW(PT_Port(pDevExt)+TX_FIFO_THRESHOLD, pDevExt->ThresHoldCnt);
			}
		  }
			//if(!remain) 
		  if((remain==1) && ( inpW( PT_Port(pDevExt)+ TX_FIFO_COUNT ) < 2048 )) {
				outB(PT_Port(pDevExt)+FIFO_CMD, TX_FIFO_SetEnd1+(remain-1));
				//////if(send_eoi)
				if(pDevExt->seoi == 1)
					outB(PT_Port(pDevExt)+FIFO_CMD, TX_FIFO_SEOI1+(remain-1));
			
				outB (PT_Port(pDevExt)+OFIFO, pDevExt->buffer[pDevExt->dwDevRA]);
				pDevExt->dwDevRA += 1;
				e = 0xff;
				outB(PT_Port(pDevExt)+INT_CTL, (intctl&0x0f)|TX_DONE_INT_EN);

		  } else if(remain)
			  		outB(PT_Port(pDevExt)+INT_CTL, (intctl&0x0f)|TX_THRES_INT_EN|TX_DONE_INT_EN);

		}
	if( hs_status & TX_DONE_INT )
	{
		//clear_bits |= HS_CLR_HF_INT;
		//printk("HS_TX_DONE int\n");
		outB(PT_Port(pDevExt)+INT_CTL, intctl&0x0f);
		//pDevExt->io_done = 1;
		//wake_up_interruptible(&(pDevExt->sync_wq) );
	}
	if( hs_status & RX_COUNT_OVER )
	{
			unsigned long remains=0;
			int hs_status2=0;
		//clear_bits |= HS_CLR_HF_INT;
		//printk("HS_RX_COUNT_OVER int\n");

		remains = ReadData(pDevExt);
 		outB(PT_Port(pDevExt)+INT_CTL, (intctl&0x78));
 		hs_status2 = inpB( PT_Port(pDevExt)+ INT_STATUS );
 		if(hs_status2 & RX_END_INT)
 			hs_status |= RX_END_INT;
		e=1;
		//pDevExt->io_done = 1;
		//wake_up_interruptible(&(pDevExt->sync_wq) );
	}

	if( hs_status & RX_END_INT )
	{
			unsigned long remains=0;
		//clear_bits |= HS_CLR_HF_INT;
		//printk("HS_RX_DONE int\n");

		remains = ReadData(pDevExt);

 		outB(PT_Port(pDevExt)+INT_CTL, intctl&0x70);
		/////outB(PT_Port(pDevExt)+INT_EN, 0);
		set_bit(RECEIVED_END_BN, &pDevExt->state);
	      if( ( pDevExt->auxa_bits & HR_HANDSHAKE_MASK ) == HR_HLDE )
			set_bit( RFD_HOLDOFF_BN, &pDevExt->state);
		e=1;
	//	pDevExt->io_done = 1;
	      //wake_up_interruptible(&(pDevExt->sync_wq) );
	}
	outB(PT_Port(pDevExt)+INT_CLR, hs_status);
  }//int 2
	if((pDevExt->dwDevRB && (((e!=0xff) && (pDevExt->dwDevRA == pDevExt->dwDevRB)) || (e==1))) || 
		(hs_status & TX_DONE_INT)) 
		{
		////IoRequestDpc((PDEVICE_OBJECT)Context,NULL, (PVOID)wRetCode);
//			printk("done\n");
			pDevExt->io_done = 1;
			wake_up_interruptible(&(pDevExt->sync_wq) );
		}
	//yuan add 01/25/08
    	/***if(res == 2 ) {
    		BOOLEAN inserted;
    		inserted = KeInsertQueueDpc(&pDevExt->ADSCDpc, NULL, NULL); 	
    	}
    		//yuan add 01/25/08
    	if(res == 3 ) {
    		BOOLEAN inserted;
    		inserted = KeInsertQueueDpc(&pDevExt->ADSCLADpc, NULL, NULL); 	
    	}****/

  return IRQ_HANDLED;
}


int ReadData(PPCI_DEVEXT pDevExt)
{
	unsigned short cnt=0, cnt_dw=0;
	unsigned long remains=0, flags;
		
	spin_lock_irqsave( &pDevExt->spinlock, flags );
	cnt = inpW( PT_Port(pDevExt)+ RX_FIFO_COUNT ) ;
	if(pDevExt->dwDevRB && (pDevExt->dwDevRB>pDevExt->dwDevRA))
		remains = pDevExt->dwDevRB - pDevExt->dwDevRA;
	else 
	{
		spin_unlock_irqrestore( &pDevExt->spinlock, flags );
		return 0;
        }
	//printk("%hd %u\n", cnt, remains);
	if(remains < cnt)
	   cnt = remains;
      cnt_dw = cnt/4;
      if(cnt_dw) {
		insl(
			PT_Port(pDevExt)+IFIFO,
			(u32 *) &(pDevExt->buffer[pDevExt->dwDevRA]),
			cnt_dw);
		pDevExt->dwDevRA += cnt_dw*4;
	}
	cnt_dw = cnt%4;
	if(cnt_dw) {
		insb(
			PT_Port(pDevExt)+IFIFO,
			(u8 *) &(pDevExt->buffer[pDevExt->dwDevRA]),
			cnt_dw);
		pDevExt->dwDevRA += cnt_dw;
	}
	remains = pDevExt->dwDevRB - pDevExt->dwDevRA;
	spin_unlock_irqrestore( &pDevExt->spinlock, flags );

	return remains;
}

