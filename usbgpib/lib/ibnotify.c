
#include "ib_internal.h"
#include <pthread.h>
#include <sys/time.h>
#include <sched.h>

void *do_notify( void *varg )
{
    ibConf_t *conf;
    struct notify_operation *asyno;
    int mask, clear_mask;
    int status, r=0;
    int retval, result=0;
    double take_msec, timeout_u=0.0;
    struct timeval tv_start, tv_end;
    struct timezone tz;
    
	//char devStr[100];
    tz.tz_minuteswest = 0;
    conf = (ibConf_t *) varg;
    asyno = &(conf->asyno);

	mask = asyno->mask;
	if(!conf->settings.usec_timeout) {
		mask &= ~TIMO;
		//timeout_u = 0xffffffff;
	} else
		timeout_u = (double) ((conf->settings.usec_timeout>=LINETIMEOUT)? (conf->settings.usec_timeout+ADDTIMEOUT):conf->settings.usec_timeout);
        
	//mask = asyno->mask;
	do {
	 clear_mask = mask & ( DTAS | DCAS );
         if(mask & TIMO)
           gettimeofday( &tv_start, &tz );
	 do {
	  retval = _ibwait( conf, mask, clear_mask, 0, &status );
	  if( retval < 0 ) 		 
	  {
	   r=-1;
	   pthread_exit(&r);
	   return NULL;//-1;
	  }
	  if(mask & TIMO) 
	  {
	   gettimeofday( &tv_end, &tz );
	   take_msec = (tv_end.tv_sec*1000+((double)(tv_end.tv_usec)/1000.0))\
			-(tv_start.tv_sec*1000+((double)(tv_start.tv_usec)/1000.0));
	   if(take_msec > timeout_u) {//conf->settings.usec_timeout)
		  status |= TIMO;
		  //yuan add 03/30/07
		  setIbsta( status );
	   }
	   //if(!(status & mask))
	   usleep(1000);		
	  }
	  if((!(status & mask)) && mask && (!conf->asyno.abort))
		//Sleep(1);
		usleep(1000);
	  else 
		  break;
	 } while (1);//((!(status & mask)) && mask && (!conf->asyno.abort));
         if(conf->asyno.abort) {
		 conf->asyno.abort = 0;
		 break;
	 }
   	 if(conf->asyno.callback) {	    	
	    	pthread_mutex_lock( &conf->asyno.lock );
		result = 0;
		ibstatus( conf, 0, 0, 0 );
    		//my_wait( conf, 0, 0, 0, &st );
		sync_globals();
		result = conf->asyno.callback(conf->asyno.ud, status, ThreadIberr(), ThreadIbcntl(), conf->asyno.cbRefData);
	  	pthread_mutex_unlock( &conf->asyno.lock );
		if(!result) 
		{
			break;
		} else
		  mask = result;
	 }
	 usleep(1000);
	} while(!conf->asyno.abort);
	asyno->mask = result;
	conf->asyno.thread = 0;
        r=0;
	pthread_exit(&r);	
	//ExitThread(0);
	return NULL;
}

int ibnotify (int ud, int mask, GpibNotifyCallback_t Callback, void *RefData)
{
	ibConf_t *conf;	
	pthread_attr_t attributes;   	
	//char devStr[100];

	conf = _func_init( ud, 1, 0 );
	if( conf == NULL )
	{
		return _func_exit( ud, 1, 0, 0, 0, 0, 1 );		
	}
	//add 01/23/06
	if((!mask) || (!Callback)) {
	 if(conf->asyno.callback)
		 	 conf->asyno.abort = 1;
	 pthread_mutex_lock( &conf->async.lock );
	 conf->async.callback = 0;
	 conf->async.cbRefData = 0;
	 pthread_mutex_unlock( &conf->async.lock );
	 pthread_mutex_lock( &conf->asyno.lock );
	 //clear_mask = mask & ( DTAS | DCAS );
	 //if(conf->asyno.callback)
	//	 	 conf->asyno.abort = 1;
	 conf->asyno.callback = 0;
	 conf->asyno.cbRefData = 0;
	 conf->asyno.mask = 0;
	 conf->asyno.thread = 0;
	 pthread_mutex_unlock( &conf->asyno.lock );
	 goto notify_end;

	}
	if(((mask & END) == END) || ((mask & CMPL) == CMPL)) { 	
	 pthread_mutex_lock( &conf->async.lock );
	 //clear_mask = mask & ( DTAS | DCAS );
	 conf->async.callback = Callback;
	 conf->async.cbRefData = RefData;
	 pthread_mutex_unlock( &conf->async.lock );
	}
	//yuan add 01/23/06
	switch(mask) {
	case TIMO:
	case SRQI:
	case RQS:
	case LOK:
	case REM:
	case CIC:
	case ATN:
	case TACS:
	case LACS:
	case DTAS:
	case DCAS:
	 pthread_mutex_lock( &conf->asyno.lock );
	 conf->asyno.callback = Callback;
	 conf->asyno.cbRefData = RefData;
	 conf->asyno.ud = ud;
	 conf->asyno.mask = mask;

	 if(!conf->asyno.thread)
	 //	conf->asyno.thread = CreateThread(NULL, 0, do_notify, conf, 0, &threadID);
	 { 
int ret =0;
	pthread_attr_init( &attributes );
	pthread_attr_setstacksize( &attributes, 0x10000 );
	ret = pthread_create( &conf->asyno.thread, &attributes, do_notify, conf );

        ret = pthread_attr_destroy( &attributes );

	 }
	 pthread_mutex_unlock( &conf->asyno.lock );
	 break;
	}
notify_end:
	return ibsta;
}
