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

static const key_t shm_msg_key = 10611;
static const key_t shm_rd_key = 10619;

typedef struct SHMmsgs
{
    volatile bool shutdown; //true for exit

    std::atomic<int> statusCount;

    std::atomic<bool> statusWriting;
    std::atomic<bool> triggerS1;

    float pos[MODEL_DOF];
    float posExt[MODEL_DOF];
    float vel[MODEL_DOF];
    float torqueActual[MODEL_DOF];

    std::atomic<bool> imuWriting;
    float pos_virtual[7]; //virtual pos(3) + virtual quat(4)
    float vel_virtual[6]; //virtual vel(3) + virtual twist(3)
    float imu_acc[3];


} SHMmsgs;

#endif // SHM_MSGS_H