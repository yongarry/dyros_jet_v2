#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

class StateManager
{
public:
    StateManager(DataContainer &dc_global);
    ~StateManager();

    void *StateThread();
    static void *ThreadStarter(void *context) { return ((StateManager *)context)->StateThread(); }

    DataContainer &dc_global_;
    RobotData &rd_global_;
    RobotData rd_local_;
};



#endif // STATE_MANAGER_H