#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include <iostream>
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "math_type_define.h"
#include "rclcpp/rclcpp.hpp"
#include "shm_msgs.h"

using namespace std;

struct RobotData
{
    ~RobotData() { std::cout << "rd terminate" << std::endl; }
    std::atomic<float> control_time_;
    std::atomic<int64_t> control_time_us_;
    std::chrono::steady_clock::time_point tp_state_;

    RigidBodyDynamics::Model model_;

    Eigen::VectorQd q_;
    Eigen::VectorQd q_ext_;
    Eigen::VectorQd q_dot_;
    Eigen::VectorQd torque_;

    Eigen::VectorQVQd q_virtual_;
    Eigen::VectorVQd q_dot_virtual_;
    Eigen::VectorVQd q_ddot_virtual_;

    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double yaw_init = 0;

    Eigen::Vector3d imu_lin_acc;

};

struct DataContainer
{
    ~DataContainer() { std::cout << "DC terminate" << std::endl; }
    RobotData rd_;
    rclcpp::Node::Node::SharedPtr nh;

    bool simMode = false;
    SHMmsgs *jc_shm_; // jet controller shared memory

    atomic<int> stm_cnt;


};

const std::string cred("\033[0;31m");
const std::string creset("\033[0m");
const std::string cblue("\033[0;34m");
const std::string cgreen("\033[0;32m");
const std::string cyellow("\033[0;33m");
#endif // ROBOT_DATA_H