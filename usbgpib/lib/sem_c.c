/* sem.c */
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/shm.h>
#include "sem_h.h"

pthread_mutex_t * semaphore_create(char *semaphore_name, int *shm_id)
{
    pthread_mutex_t* lock_m;
    int m_shmid=0;
    key_t semkey=0;
    pthread_mutexattr_t psharedm;
    
    semkey = (key_t) strtol(semaphore_name, NULL, 35);
    //printf("key_t: %d\n", semkey);
    m_shmid=shmget(semkey, sizeof(pthread_mutex_t), 0666|IPC_CREAT|IPC_EXCL);
    if(m_shmid==-1) 
    {
	if(errno == EEXIST)
	{
	  m_shmid = shmget(semkey, sizeof(pthread_mutex_t), 0666);	
	  if(m_shmid ==-1)
	  {
		printf("0: share create failed %d\n", errno);
		return 0;
	  } 
	  else
	  {
		printf("0:shmid %x\n",m_shmid );		
		lock_m = (pthread_mutex_t *) shmat(m_shmid,0,0);	
		if( lock_m == ((pthread_mutex_t*) -1)) 
		{
		printf("mutex object creation failed\n");
		return 0;
		}
		*shm_id = m_shmid;
		return lock_m; 
	  }
	} else 
	{
		//printf("00: share create failed %d\n", errno);
		return 0;
	}
    }		
    else
    {
	//printf("00:shmid %x\n",m_shmid );
	lock_m =(pthread_mutex_t *) shmat(m_shmid,0,0);
    	pthread_mutexattr_setpshared(&psharedm, PTHREAD_PROCESS_SHARED);
	if(lock_m == ((pthread_mutex_t*) -1)) 
	{
		printf("mutex object creation failed\n");
		return 0;
	}
        pthread_mutexattr_init(&psharedm);
	pthread_mutexattr_setpshared(&psharedm, PTHREAD_PROCESS_SHARED);
	pthread_mutex_init(lock_m, &psharedm);
	//printf("lock %x\n", lock_m );	
	*shm_id = m_shmid; 	
    }
    return lock_m;
}

void
semaphore_close(pthread_mutex_t *semap, int m_shmid)
{
    	shmdt((char*) semap);
	shmctl(m_shmid, IPC_RMID, (struct shmid_ds *)0);    
}
