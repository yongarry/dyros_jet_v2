#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

struct DataContainer
{
    ~DataContainer() { std::cout << "DC terminate" << std::endl; }
    RobotData rd_;
 
    bool simMode = false;
    SHMmsgs *jc_shm_; // jet controller shared memory

};

struct RobotData
{
    ~RobotData() { std::cout << "rd terminate" << std::endl; }
};