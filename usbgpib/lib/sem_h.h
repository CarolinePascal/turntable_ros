/* sem_h.h */
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/sem.h>

pthread_mutex_t * semaphore_create(char *semaphore_name, int *shm_id);
void semaphore_close(pthread_mutex_t *semap, int m_shmid);
