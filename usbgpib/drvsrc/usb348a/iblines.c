/***************************************************************************
 iblines.c     
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from cb7210/cb7210_aux.c of the Linux GPIB Package driver 
 							by Frank Mori Hess      
 Copyright (C) 2002 by Frank Mori Hess <fmhess@users.sourceforge.net>
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
 
#include "gpib_proto.h"
#include "gpib_user.h"
#include "../include/Adllib.h"
#include "u348a.h"

short iblines( const PU348A_DEVEXT board )
{
	int status = ValidALL;
	int bsr_bits;
	
   bsr_bits = paged_read_byte( board, BUS_STATUS, BUS_STATUS_PAGE );
	 if( bsr_bits & BSR_REN_BIT_9607 )
		status |= BusREN;
	 if( bsr_bits & BSR_IFC_BIT_9607 )
		status |= BusIFC;
	 if( bsr_bits & BSR_SRQ_BIT_9607 )
		status |= BusSRQ;
	 if( bsr_bits & BSR_EOI_BIT_9607 )
		status |= BusEOI;
	 if( bsr_bits & BSR_NRFD_BIT_9607 )
		status |= BusNRFD;
	 if( bsr_bits & BSR_NDAC_BIT_9607 )
		status |= BusNDAC;
	 if( bsr_bits & BSR_DAV_BIT_9607 )
		status |= BusDAV;
	 if( bsr_bits & BSR_ATN_BIT_9607 )
		status |= BusATN;	    
	//printk("lines: %x %x\n", bsr_bits, status);
	return ((short) status);
}
