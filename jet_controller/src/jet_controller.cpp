#include "jet_controller/jet_controller.h"
using namespace std;

JetController::JetController(StateManager &stm_global) : dc_(stm_global.dc_), stm_(stm_global), rd_(stm_global.dc_.rd_)
#ifdef COMPILE_JET_CC
                                                        , my_cc(*(new CustomController(rd_gobal_)))
#endif
{

}

JetController::~JetController() { cout << cgreen <<"Jet Controller Terminated" << creset <<endl; }

void *JetController::Thread1() // Hard Real Time with 100Hz
{
}

void *JetController::Thread2() // Soft Real Time with 100Hz ComputeFast
{
}

void *JetController::Thread3() // Soft Real Time with 100Hz ComputeSlow
{
}


