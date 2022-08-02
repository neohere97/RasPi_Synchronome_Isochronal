#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>

#define NUM_THREADS 100

typedef struct
{
    int threadIdx;
    int dump;
} threadParams_t;


// POSIX thread declarations and scheduling attributes
//
pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];


// void *counterThread(void *threadp)
// {
//     int sum=0, i;
//     threadParams_t *threadParams = (threadParams_t *)threadp;

//     for(i=1; i < (threadParams->threadIdx)+1; i++)
//         sum=sum+i;
 
//     printf("Thread idx=%d, sum[0...%d]=%d\n", 
//            threadParams->threadIdx,
//            threadParams->threadIdx, sum);
// }

void *printer_thread(void *threadparams){
    threadParams_t *threadParams = (threadParams_t)threadparams;

    printf("Thread Number is %d \n Thread Dump Val is %d \n\n",threadParams->threadIdx,threadParams->dump);
}


int main (int argc, char *argv[])
{
   int rc;
   int i;

   for(i=0; i < NUM_THREADS; i++)
   {
       threadParams[i].threadIdx=i;
       threadParams[i].dump = i%2;

       pthread_create(&threads[i],   // pointer to thread descriptor
                      (void *)0,     // use default attributes
                      printer_thread, // thread function entry point
                      (void *)&(threadParams[i]) // parameters to pass in
                     );

   }

   for(i=0;i<NUM_THREADS;i++)
       pthread_join(threads[i], NULL);

   printf("TEST COMPLETE\n");
}