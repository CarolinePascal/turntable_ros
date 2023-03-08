/***************************************************************************
 ibConf.c
        
 Copyright (C) 2018 ADLINK Corporation.
	  
 adapted from lib/ibConf.c of the Linux GPIB Package driver 
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

static int set_spoll_timeout( ibConf_t *conf, int timeout )
{
	if( timeout < TNONE || timeout > T1000s )
	{
		setIberr( EARG );
		return -1;
	}

	conf->settings.spoll_usec_timeout = timeout_to_usec( timeout );

	return 0;
}

static int set_ppoll_timeout( ibConf_t *conf, int timeout )
{
	if( timeout < TNONE || timeout > T1000s )
	{
		setIberr( EARG );
		return -1;
	}

	conf->settings.ppoll_usec_timeout = ppoll_timeout_to_usec( timeout );

	return 0;
}

int set_t1_delay( ibBoard_t *board, int delay )
{
	t1_delay_ioctl_t nano_sec;
	int retval;

	switch( delay )
	{
		case T1_DELAY_2000ns:
			nano_sec = 2000;
			break;
		case T1_DELAY_500ns:
			nano_sec = 500;
			break;
		case T1_DELAY_350ns:
			nano_sec = 350;
			break;
		default:
			setIberr( EARG );
			return -1;
			break;
	}

	retval = ioctl( board->fileno, IB_T1_DELAY, &nano_sec );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return -1;
	}

	return 0;
}

int set_ibLon( ibBoard_t *board, signed int code)
{
	online_ioctl_t online_cmd;
    	int retval = 0;

	online_cmd.online = code;//3;

retval = ioctl( board->fileno, IBONL, &online_cmd );
	if( retval < 0 )
	{
		setIberr( EDVR );
		setIbcnt( errno );
		return -1;
	}

	//here also set the EOS
	return retval;
}

int ibconfig( int ud, int option, int value )
{
	ibConf_t *conf;	
        int retval, result =0;
	conf = func_init( ud );
	if( conf == NULL )
		return func_exit( ud, 1 );
	pthread_mutex_lock( m_lock[conf->settings.board] );
	switch( option )
	{
		case IbcPAD:
			retval = _ibpad( conf, value );
			if( retval < 0 ) 
			{
				result = func_exit( ud, 1 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;
			}
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			break;
		case IbcSAD:
			retval = _ibsad( conf, value );
			if( retval < 0 ) 
			{
				result = func_exit( ud, 1 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;
			}
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//if( retval < 0 ) return func_exit( ud, 1 );
			//return func_exit( ud, 0 );
			break;
		case IbcTMO:
			retval = _ibtmo( conf, value );
			if( retval < 0 ) 
			{
				result = func_exit( ud, 1 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;
			}
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//if( retval < 0 ) return func_exit( ud, 1 );
			//return func_exit( ud, 0 );
			break;
		case IbcEOT:
			_ibeot( conf, value );
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//return func_exit( ud, 0 );
			break;
		case IbcEOSrd:
			setIberr( (conf->settings.eos_flags & REOS)? 1:0 );		
			if( value )
				conf->settings.eos_flags |= REOS;
			else
				conf->settings.eos_flags &= ~REOS;
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//return func_exit( ud, 0 );
			break;
		case IbcEOSwrt:
			setIberr( (conf->settings.eos_flags & XEOS)? 1:0 );
			if( value )
				conf->settings.eos_flags |= XEOS;
			else
				conf->settings.eos_flags &= ~XEOS;
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//return func_exit( ud, 0 );
			break;
		case IbcEOScmp:
			setIberr( (conf->settings.eos_flags & BIN)? 1:0 );
			if( value )
				conf->settings.eos_flags |= BIN;
			else
				conf->settings.eos_flags &= ~BIN;
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//return func_exit( ud, 0 );
			break;
		case IbcEOSchar:
			if( ( value & 0xff ) != value )
			{
				setIberr( EARG );
				result = func_exit( ud, 1 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;
			}
			setIberr( conf->settings.eos );
			conf->settings.eos = value;
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//return func_exit( ud, 0 );
			break;
		case IbcReadAdjust:
			setIberr( 0 );
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//return func_exit( ud, 0 );
			break;
		case IbcWriteAdjust:
			setIberr( 0 );
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//return func_exit( ud, 0 );
			break;
		case IbcEndBitIsNormal:
			setIberr( 0 );
			result = func_exit( ud, 0 );
		        pthread_mutex_unlock( m_lock[conf->settings.board] );
			return result;
			//return func_exit( ud, 0 );
			break;
		default:
			break;
	}

	if( conf->is_interface )
	{
		switch( option )
		{
			case IbcPPC:
				retval = _ibppc( conf, value );
				if( retval < 0 ) 
				{
					result = func_exit( ud, 1 );	
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;
				}
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;
				//	return func_exit( ud, 0 );
				break;
			case IbcAUTOPOLL:
				retval = configure_autospoll( conf, value );
				if( retval < 0 ) {
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;
					//return func_exit( ud, 1 );	
				}
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;
				//return func_exit( ud, 0 );
				break;
			case IbcCICPROT:
				setIberr( 0 );
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;
				//return func_exit( ud, 0 );
				break;
			case IbcIRQ:
				setIberr( interfaceBoard(conf)->irq );
				interfaceBoard(conf)->irq = value;
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;			
				//return func_exit( ud, 0 );
				break;
			case IbcSC:
				retval = _ibrsc( conf, value );
				if( retval < 0 )
				{
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;			
					//return func_exit( ud, 1 );
				} else
				{
					result = func_exit( ud, 0 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;			
				}	
				//return func_exit( ud, 0 );
				break;
			case IbcSRE:
				retval = _ibsre( conf, value );
				if( retval < 0 ) 
				{
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;			
					//return func_exit( ud, 1 );
				}
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;			
				//return func_exit( ud, 0 );
				break;
			case IbcPP2:
				conf->settings.local_ppc = 1;
				retval = _ibppc( conf, value );
				if( retval < 0 ) 
				{
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 1 );
				}
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;			
				//return func_exit( ud, 0 );				
				break;
			case IbcTIMING:
				setIberr( ask_board_t1_delay(interfaceBoard(conf)));	
				if( set_t1_delay( interfaceBoard( conf ), value ) < 0 )
				{
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 1 );
				}
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;			
				//return func_exit( ud, 0 );
				break;
			case IbcDMA:
				setIberr( interfaceBoard(conf)->dma );
				interfaceBoard(conf)->dma = value;
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;			
				//	return func_exit( ud, 0 );
				break;
			case IbcEventQueue:
				if( value )
				{
					interfaceBoard(conf)->use_event_queue = 1;
				}else
					interfaceBoard(conf)->use_event_queue = 0;
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;			
				//return func_exit( ud, 0 );
				break;
			case IbcSPollBit:
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;			
				//return func_exit(ud, 0);
				break;
			case IbcSendLLO:
				setIberr( conf->settings.local_lockout );
				conf->settings.local_lockout = value;
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;			
				//return func_exit( ud, 0 );
				break;
			case IbcPPollTime:
				setIberr (usec_to_ppoll_timeout( conf->settings.ppoll_usec_timeout ));				retval = set_ppoll_timeout( conf, value );
				if( retval < 0 )
				{
					setIberr( EARG );
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 1 );
				}else
				{
					result = func_exit( ud, 0 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 0 );
				}
				break;
			case IbcHSCableLength:
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;		
				//return func_exit( ud, 0 );
				break;
			case IbcIst:
				retval = _ibist( conf, value );
				if( retval < 0 ) {
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 1 );
				}
				else 
				{
					result = func_exit( ud, 0 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 0 );
				}
				break;
			case IbcRsv:
				retval = _ibrsv( conf, value );
				if( retval < 0 )
				{
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 1 );
				}
				else
				{
					result = func_exit( ud, 0 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 0 );
				}
				break;
			case IbcLON:
				if( value )
				{
					if( set_ibLon( interfaceBoard( conf ) , 3) < 0 ) {
						result = func_exit( ud, 1 );
				        	pthread_mutex_unlock( m_lock[conf->settings.board] );
						return result;		
						//return func_exit( ud, 1 );
						
					}
				} 
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;		
				//return func_exit( ud, 0 );
				break;
			default:
				break;
		}
	}else
	{
		switch( option )
		{
			case IbcREADDR:
				setIberr( conf->settings.readdr );
				if( value )
					conf->settings.readdr = 1;
				else
					conf->settings.readdr = 0;
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;		
				//return func_exit( ud, 0 );
				break;
			case IbcSPollTime:
				setIberr( usec_to_timeout( conf->settings.spoll_usec_timeout ) );
				retval = set_spoll_timeout( conf, value );
				if( retval < 0 )
				{
					setIberr( EARG );				
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 1 );
				}
				else
				{
					result = func_exit( ud, 0 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 0 );
				}
				break;
			case IbcUnAddr:
				setIberr(0);
				result = func_exit( ud, 0 );
			        pthread_mutex_unlock( m_lock[conf->settings.board] );
				return result;		
				//return func_exit( ud, 0 );
				break;
			case IbcBNA:
				retval = _ibbna( conf, value );
				if( retval < 0 )
				{
					result = func_exit( ud, 1 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 1 );
				}
				else
				{
					result = func_exit( ud, 0 );
				        pthread_mutex_unlock( m_lock[conf->settings.board] );
					return result;		
					//return func_exit( ud, 0 );
				}
				//if( retval < 0 )
				//	return func_exit( ud, 1 );
				//else
				//	return func_exit( ud, 0 );
				break;
			default:
				break;
		}
	}
	setIberr( EARG );
	result = func_exit( ud, 1 );
        pthread_mutex_unlock( m_lock[conf->settings.board] );
	return result;
}
