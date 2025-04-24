#ifndef JET_CONTROLLER_H
#define JET_CONTROLLER_H

#include "jet_controller/state_manager.h"
#include "rclcpp/rclcpp.hpp"

#ifdef COMPILE_JET_CC
#include "cc.h"
#endif

class JetController
{
public:
    JetController(StateManager &stm);
    ~JetController();

    void *Thread1();
    void *Thread2();
    void *Thread3();

    DataContainer &dc_global_;
    StateManager &stm_global_;
    RobotData &rd_global_;

#ifdef COMPILE_JET_CC
    CustomController &my_cc_;
#endif

    static void *Thread1Starter(void *context) { return ((JetController *)context)->Thread1(); }
    static void *Thread2Starter(void *context) { return ((JetController *)context)->Thread2(); }
    static void *Thread3Starter(void *context) { return ((JetController *)context)->Thread3(); }

    void MeasureTime(int currentCount, int nanoseconds1, int nanoseconds2 = 0);
    int64_t total1 = 0, total2 = 0, total_dev1 = 0, total_dev2 = 0;
    float lmax = 0.0, lmin = 10000.00, ldev = 0.0, lavg = 0.0, lat = 0.0;
    float smax = 0.0, smin = 10000.00, sdev = 0.0, savg = 0.0, sat = 0.0;

    std::atomic<bool> enableThread2;
    void EnableThread2(bool enable);
    std::atomic<bool> enableThread3;
    void EnableThread3(bool enable);

    void RequestThread2();
    void RequestThread3();
    
    std::atomic<bool> signalThread1;
    std::atomic<bool> triggerThread2;
    std::atomic<bool> triggerThread3;

};

const std::string cred("\033[0;31m");
const std::string creset("\033[0m");
const std::string cblue("\033[0;34m");
const std::string cgreen("\033[0;32m");
const std::string cyellow("\033[0;33m");
#endif // JET_CONTROLLER_H