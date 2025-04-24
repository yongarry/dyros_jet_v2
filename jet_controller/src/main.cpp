/*
    Dyros Jet V2 Controller
    (c) 2025 Jaeyong Shin <jasonshin0537@snu.ac.kr>
*/

#include "jet_controller/jet_controller.h"

volatile bool *prog_shutdown;

void SIGINT_handler(int sig)
{
    cout << " CNTRL : shutdown Signal" << endl;
    *prog_shutdown = true;
}
using namespace std;

int main(int argc, char **argv)
{
    cout << endl;
    cout << "=====================================" << endl;
    cout << " CNTRL : Starting JETv2 CONTROLLER! " << endl;
    cout << "=====================================" << endl;
    signal(SIGINT, SIGINT_handler);

#ifdef COMPILE_REALROBOT
    mlockall(MCL_CURRENT | MCL_FUTURE);
#endif

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("jet_controller");

    DataContainer dc_;
    dc_.simMode = node->declare_parameter("/jet_controller/sim_mode", false);

    StateManager stm(dc_);

    JetController jc_(stm);

    int shm_id_;
    init_shm(shm_msg_key, shm_id_, &dc_.jc_shm_);
    // check if the sh
    prog_shutdown = &dc_.jc_shm_->shutdown;

    if (dc_.tc_shm_->shutdown)
    {
        cout << cred << endl;
        cout << "Shared memory was not successfully removed from the previous run. " << endl;
        cout << "Please Execute shm_reset : rosrun jet_controller shm_reset " << endl;
        cout << "Or you can remove reset shared memory with 'sudo ipcrm -m " << shm_id_ << "'" << creset << std::endl;
    }
    else
    {
        const int thread_number = 4;

        struct sched_param param_st;
        struct sched_param param;
        struct sched_param param_controller;
        struct sched_param param_logger;
        pthread_attr_t attrs[thread_number];
        pthread_t threads[thread_number];
        param.sched_priority = 42 + 50;
        param_logger.sched_priority = 41 + 50;
        param_controller.sched_priority = 45 + 50;
        param_st.sched_priority = 45 + 50;
        cpu_set_t cpusets[thread_number];

        if (dc_.simMode)
            cout << "Simulation Mode" << endl;


        /* Initialize pthread attributes (default values) */
        for (int i = 0; i < thread_number; i++)
        {
            if (pthread_attr_init(&attrs[i]))
            {
                printf("attr %d init failed ", i);
            }

            if (!dc_.simMode)
            {
                if (pthread_attr_setschedpolicy(&attrs[i], SCHED_FIFO))
                {
                    printf("attr %d setschedpolicy failed ", i);
                }
                if (pthread_attr_setschedparam(&attrs[i], &param))
                {
                    printf("attr %d setschedparam failed ", i);
                }
                if (pthread_attr_setinheritsched(&attrs[i], PTHREAD_EXPLICIT_SCHED))
                {
                    printf("attr %d setinheritsched failed ", i);
                }
            }
        }

        pthread_attr_init(&loggerattrs);
        
        if (!dc_.simMode)
        {
            if (pthread_attr_setschedparam(&attrs[0], &param_st))
            {
                printf("attr %d setschedparam failed ", 0);
            }
            
            if (pthread_attr_setschedparam(&attrs[1], &param_controller))
            {
                printf("attr %d setschedparam failed ", 0);
            }

            CPU_ZERO(&cpusets[0]);
            CPU_SET(5, &cpusets[0]);
            if (pthread_attr_setaffinity_np(&attrs[0], sizeof(cpu_set_t), &cpusets[0]))
            {
                printf("attr %d setaffinity failed ", 0);
            }

            if (pthread_attr_setschedpolicy(&loggerattrs, SCHED_FIFO))
            {
                printf("attr logger setschedpolicy failed ");
            }
            if (pthread_attr_setschedparam(&loggerattrs, &param_logger))
            {
                printf("attr logger setschedparam failed ");
            }
            if (pthread_attr_setinheritsched(&loggerattrs, PTHREAD_EXPLICIT_SCHED))
            {
                printf("attr logger setinheritsched failed ");
            }
        }

        if (pthread_create(&threads[0], &attrs[0], &StateManager::ThreadStarter, &stm))
        {
            printf("threads[0] create failed\n");
        }
        if (pthread_create(&threads[1], &attrs[1], &JetController::Thread1Starter, &jc_))
        {
            printf("threads[1] create failed\n");
        }
        if (pthread_create(&threads[2], &attrs[2], &JetController::Thread2Starter, &jc_))
        {
            printf("threads[2] create failed\n");
        }
        if (pthread_create(&threads[3], &attrs[3], &JetController::Thread3Starter, &jc_))
        {
            printf("threads[3] create failed\n");
        }
        for (int i = 0; i < thread_number; i++)
        {
            pthread_attr_destroy(&attrs[i]);
        }

        /* Join the thread and wait until it is done */
        for (int i = 0; i < thread_number; i++)
        {
            pthread_join(threads[i], NULL);
        }
    }
    rclcpp::shutdown();
    deleteSharedMemory(shm_id_, dc_.jc_shm_);
    cout << cgreen << " CNTRL : jet controller Shutdown" << creset << endl;
    return 0;
}