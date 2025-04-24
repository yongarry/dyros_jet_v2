#ifndef SHM_MSGS_H
#define SHM_MSGS_H

#include <pthread.h>
#include <atomic>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <time.h>

#if defined(__x86_64) || defined(__i386)
#define cpu_relax() __asm__("pause" :: \
                                : "memory")
#else
#define cpu_relax() __asm__("" :: \
                                : "memory")
#endif

#define mb()        asm volatile("mfence" : : : "memory")

#define MODEL_DOF 12

typedef struct SHMmsgs
{
    volatile bool shutdown; //true for exit

} SHMmsgs;