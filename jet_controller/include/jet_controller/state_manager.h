#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "jet_lib/robot_data.h"
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define JET_DOF 12

class StateManager
{
public:
    StateManager(DataContainer &dc_global);
    ~StateManager();

    void *StateThread();
    static void *ThreadStarter(void *context) { return ((StateManager *)context)->StateThread(); }

    DataContainer &dc_;
    RobotData &rd_;
    RobotData rd_local_;

    RigidBodyDynamics::Model model_local_, model_global_;

    // Ros Communication
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mujoco_sim_command_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mujoco_sim_command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    sensor_msgs::msg::JointState joint_state_msg_;
    void SimCommandCallback(const std_msgs::msg::String::SharedPtr msg);


    float control_time_;

    void GetJointData();
    // for getting from shared memory we need array var
    float q_a_[JET_DOF] = {};
    float q_dot_a_[JET_DOF] = {};
    float torqueActual_a_[JET_DOF] = {};
    float q_ext_a[JET_DOF] = {};

    Eigen::VectorQd q_;
    Eigen::VectorQd q_dot_;
    Eigen::VectorQd q_ext_;
    Eigen::VectorQd torque_;

    Eigen::VectorQVQd q_virtual_;
    Eigen::VectorVQd q_dot_virtual_;
    Eigen::VectorVQd q_ddot_virtual_;

    Eigen::VectorQVQd q_virtual_local_;
    Eigen::VectorVQd q_dot_virtual_local_;
    Eigen::VectorVQd q_ddot_virtual_local_;
    Eigen::VectorQVQd q_virtual_local_yaw_initialized;

    // imu pos estimate
    Eigen::Vector3d imu_lin_acc_lpf;
    Eigen::Vector3d imu_lin_acc_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_lin_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_pos_ = Eigen::Vector3d::Zero();

    void UpdateYaw();
    void GetSensorData();
    void StateEstimate();
    void StoreState(RobotData &rd_global_);
    void SendCommand();

};

namespace JET
{
    const std::string JOINT_NAME[JET_DOF] = {
        "L_HipYaw", "L_HipRoll", "L_HipPitch", "L_KneePitch", "L_AnklePitch", "L_AnkleRoll",
        "R_HipYaw", "R_HipRoll", "R_HipPitch", "R_KneePitch", "R_AnklePitch", "R_AnkleRoll"};
    
    
};

#endif // STATE_MANAGER_H