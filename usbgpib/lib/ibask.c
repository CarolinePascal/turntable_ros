/***************************************************************************
 ibask.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibask.c of the Linux GPIB Package driver 
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
 
#include "ib_internal.h"

int ask_ist( const ibBoard_t *board )
{
	int retval;
	board_info_ioctl_t info;

	retval = ioctl( board->fileno, IBBOARD_INFO, &info );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	return info.ist;
}

int ask_ppc( const ibBoard_t *board )
{
	int retval;
	board_info_ioctl_t info;

	retval = ioctl( board->fileno, IBBOARD_INFO, &info );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	return info.parallel_poll_configuration;
}

int ask_autopoll( const ibBoard_t *board )
{
	int retval;
	board_info_ioctl_t info;

	retval = ioctl( board->fileno, IBBOARD_INFO, &info );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	return info.autopolling;
}

int ask_board_t1_delay( const ibBoard_t *board )
{
	int retval;
	board_info_ioctl_t info;

	retval = ioctl( board->fileno, IBBOARD_INFO, &info );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

        if( info.t1_delay < 500 ) return T1_DELAY_350ns;
	else if( info.t1_delay < 2000 ) return T1_DELAY_500ns;
	return T1_DELAY_2000ns;
}

int ask_board_rsv( const ibBoard_t *board )
{
	int retval;
	int status;

	retval = ioctl( board->fileno, IBQUERY_BOARD_RSV, &status );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	return status;
}

int ask_pad( const ibBoard_t *board, unsigned int *pad )
{
	int retval;
	board_info_ioctl_t info;

	retval = ioctl( board->fileno, IBBOARD_INFO, &info );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	*pad = info.pad;
	return 0;
}

int ask_sad( const ibBoard_t *board, int *sad )
{
	int retval;
	board_info_ioctl_t info;

	retval = ioctl( board->fileno, IBBOARD_INFO, &info );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return retval;
	}

	*sad = info.sad;
	return 0;
}

int ibask( int ud, int option, int *value )
{
	ibConf_t *conf;
	ibBoard_t *board;
	int retval;

	conf = _func_init( ud, 1, 0 );
	if( conf == NULL )
		return func_exit( ud, 1);

	board = interfaceBoard( conf );

	switch( option )
	{
		case IbaPAD:
			if( conf->is_interface )
			{
				unsigned int pad;

				retval = ask_pad( board, &pad );
				if( retval < 0 ) return func_exit( ud, 1 );
				conf->settings.pad = pad;
			}
			*value = conf->settings.pad;
			return func_exit( ud, 0 );
			break;
		case IbaSAD:
			if( conf->is_interface )
			{
				int sad;

				retval = ask_sad( board, &sad );
				if( retval < 0 ) return func_exit( ud, 1 );
				conf->settings.sad = sad;
			}
			if( conf->settings.sad < 0 ) *value = 0;
			else *value = MSA( conf->settings.sad );
			return func_exit( ud, 0 );
			break;
		case IbaTMO:
			*value = usec_to_timeout( conf->settings.usec_timeout );
			return func_exit( ud, 0 );
			break;
		case IbaEOT:
			*value = conf->settings.send_eoi;
			return func_exit( ud, 0 );
			break;
		case IbaEOSrd:
			*value = conf->settings.eos_flags & REOS;
			return func_exit( ud, 0 );
			break;
		case IbaEOSwrt:
			*value = conf->settings.eos_flags & XEOS;
			return func_exit( ud, 0 );
			break;
		case IbaEOScmp:
			*value = conf->settings.eos_flags & BIN;
			return func_exit( ud, 0 );
			break;
		case IbaEOSchar:
			*value = conf->settings.eos;
			return func_exit( ud, 0 );
			break;
		case IbaReadAdjust:
			*value = 0;
			return func_exit( ud, 0 );
			break;
		case IbaWriteAdjust:
			*value = 0;
			return func_exit( ud, 0 );
			break;
		case IbaEndBitIsNormal:
			*value = 1;
			return func_exit( ud, 0 );
			break;
		default:
			break;
	}

	if( conf->is_interface )
	{
		switch( option )
		{
			case IbaPPC:
				retval = ask_ppc( board );
				if( retval < 0 ) return func_exit( ud, 1 );
				*value = retval;
				return func_exit( ud, 0 );
				break;
			case IbaAUTOPOLL:
				retval = ask_autopoll( board );
				if( retval < 0 ) return func_exit( ud, 1 );
				*value = retval;
				return func_exit( ud, 0 );
				break;
			case IbaCICPROT:
				*value = 0;
				return func_exit( ud, 0 );
				break;
			case IbaIRQ:
				*value = 0;
				return func_exit( ud, 0 );
				break;
			case IbaSC:
				retval = check_sc( board );
				if( retval < 0 ) return func_exit( ud, 1 );
				*value = retval;
				return func_exit( ud, 0 );
				break;
			case IbaSRE:
				*value = 1;
				return func_exit( ud, 0 );
				break;
			case IbaPP2:
				*value = conf->settings.local_ppc;
				return func_exit( ud, 0 );
				break;
			case IbaTIMING:
				retval = ask_board_t1_delay( board );
				if( retval < 0 ) return func_exit( ud, 1 );
				*value = retval;
				return func_exit( ud, 0 );
				break;
			case IbaDMA:
				*value = board->dma;
				return func_exit( ud, 0 );
				break;
			//case IbaEventQueue:
			//	*value = board->use_event_queue;
			//	return func_exit( ud, 0 );
			//	break;
			//case IbaSPollBit:
			//	*value = 0;
			//	return func_exit( ud, 0 );
			//	break;
			case IbaSendLLO:
				*value = conf->settings.local_lockout;
				return func_exit( ud, 0 );
				break;
			case IbaPPollTime:
				*value = usec_to_ppoll_timeout( conf->settings.ppoll_usec_timeout );
				return func_exit( ud, 0 );
				break;
			case IbaHSCableLength:
				*value = 0;
				return func_exit( ud, 0 );
				break;
			case IbaIst:
				retval = ask_ist( board );
				if( retval < 0 ) return func_exit( ud, 1 );
				*value = retval;
				return func_exit( ud, 0 );
			break;
			case IbaRsv:
				retval = ask_board_rsv( board );
				if( retval < 0 ) return func_exit( ud, 1 );
				*value = retval;
				return func_exit( ud, 0 );
				break;
			case IbaBaseAddr:
				//GetGPIBBoardIO(ud, value);
				*value = ibBoard[ud].base[1] & 0xfffffffc;
				return func_exit( ud, 0 );		
			/*case Iba7BitEOS:
				retval = query_no_7_bit_eos(board);
				if( retval < 0 ) return func_exit( ud, 1 );
				*value = !retval;
				return func_exit( ud, 0 );
				break;*/
			default:
				break;
		}
	}else
	{
		switch( option )
		{
			case IbaREADDR:
				*value = conf->settings.readdr;
				return func_exit( ud, 0 );
				break;
			case IbaSPollTime:
				*value = usec_to_timeout( conf->settings.spoll_usec_timeout );
				return func_exit( ud, 0 );
				break;
			case IbaUnAddr:
				*value = 0;
				return func_exit( ud, 0 );
				break;
			case IbaBNA:
				*value = conf->settings.board;
				return func_exit( ud, 0 );
				break;
			default:
				break;
		}
	}

	setIberr( EARG );

	return func_exit( ud, 1 );
}


