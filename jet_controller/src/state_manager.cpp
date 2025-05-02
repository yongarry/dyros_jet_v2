#include "jet_controller/state_manager.h"

using namespace std;

StateManager::StateManager(DataContainer &dc_global) : dc_(dc_global), rd_(dc_global.rd_)
{
    string urdf_path;
    dc_.nh->get_parameter("/jet_controller/urdf", urdf_path);

    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_local_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_global_, true, false);
    //check model dof size
    if (model_local_.q_size != JET_DOF)
    {
        std::cout << "MODEL DOF SIZE NOT MATCH" << std::endl;
    }

    if (dc_.simMode) // for mujoco simulation ctrl
    {
        mujoco_sim_command_pub_ = dc_.nh->create_publisher<std_msgs::msg::String>("/mujoco_ros_interface/sim_command_con2sim", 1);
        mujoco_sim_command_sub_ = dc_.nh->create_subscription<std_msgs::msg::String>("/mujoco_ros_interface/sim_command_sim2con", 1, std::bind(&StateManager::SimCommandCallback, this, std::placeholders::_1));
    }

    joint_state_pub_ = dc_.nh->create_publisher<sensor_msgs::msg::JointState>("/jet/joint_states", 1);
    joint_state_msg_.name.resize(model_local_.q_size);
    joint_state_msg_.position.resize(model_local_.q_size);
    joint_state_msg_.velocity.resize(model_local_.q_size);
    joint_state_msg_.effort.resize(model_local_.q_size);
    for (unsigned int i = 0; i < model_local_.q_size; i++)
        joint_state_msg_.name[i] = JET::JOINT_NAME[i];

}

StateManager::~StateManager()
{
    cout << cgreen << "State Manager Terminated" << creset << endl;
}

void *StateManager::StateThread()
{
    // Wait for the other thread to start or initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cout << " STATE : started with pid : " << getpid() << std::endl;

    int rcv_cnt = -1;
    // Check if the shared memory is initialized
    rcv_cnt = dc_.jc_shm_->statusCount;
    int cycle_count_ = rcv_cnt;
    dc_.stm_cnt = 0;

    int cnt = 0;
    int cnt2 = 0;
    int cnt3 = 0;
    auto time_start = std::chrono::steady_clock::now();

    timespec tv_us1;
    tv_us1.tv_sec = 0;
    tv_us1.tv_nsec = 10000;

    while(true)
    {
        if (dc_.jc_shm_->shutdown)
            break;

        auto t0 = std::chrono::steady_clock::now();
        while (!dc_.jc_shm_->triggerS1)
        {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &tv_us1, NULL);
            // prevent Memory Optimization for intended execution flow
            __asm__("pause" ::
                        : "memory");
            if (dc_.jc_shm_->shutdown)
                break;
        }
        rd_local_.tp_state_ = std::chrono::steady_clock::now();
        auto t1 = rd_local_.tp_state_;
        if (chrono::duration_cast<chrono::microseconds>(t1 - t0).count() > 500)
        {
            if (control_time_ > 0.5)
            {
                if (!dc_.jc_shm_->shutdown)
                {
                    if (!dc_.simMode)
                    {
                        cout << cyellow;
                        cout << " STATE : Waiting for signal for over 500us, " << chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
                        cout << " at, " << control_time_ << creset << std::endl;
                    }
                }
            }
        }
        dc_.jc_shm_->triggerS1 = false;
        cycle_count_++;
        dc_.stm_cnt++;

        rcv_cnt = dc_.jc_shm_->statusCount;

        GetJointData(); 
        UpdateYaw();

        // auto d1 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t1).count();
        // auto t2 = chrono::steady_clock::now();

        // auto dur_start_ = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - time_start).count();
        control_time_ = rcv_cnt / 2000.0;

        // auto d2 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t2).count();
        // auto t3 = chrono::steady_clock::now();
        // UpdateKinematics_local(model_local_, link_local_, q_virtual_local_, q_dot_virtual_local_, q_ddot_virtual_local_);

        GetSensorData();

        // auto d3 = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t3).count();
        // auto t4 = chrono::steady_clock::now();
        StateEstimate();
        // UpdateKinematics(model_global_, link_, q_virtual_, q_dot_virtual_, q_ddot_virtual_);

        StoreState(rd_);

        rd_.control_time_ = dur_start_ / 1000000.0;
        rd_.control_time_us_ = dur_start_;
        dc_.jc_shm_->control_time_us_ = dur_start_;
        control_time_us_l_ = dur_start_;

        dc_.jc_shm_->stloopCount.store(dc_.stm_cnt);

        SendCommand();
    }
    cout << " STATE : StateManager END" << endl;
    return (void *)NULL;
}

void StateManager::GetJointData()
{
    while (dc_.jc_shm_->statusWriting.load(std::memory_order_acquire))
    {
        usleep(10);
        if (dc_.jc_shm_->shutdown)
            break;
    }

    // get data from shared memory in to array
    memcpy(q_a_, dc_.jc_shm_->pos, sizeof(float) * JET_DOF);
    memcpy(q_dot_a_, dc_.jc_shm_->vel, sizeof(float) * JET_DOF);
    memcpy(torqueActual_a_, dc_.jc_shm_->torqueActual, sizeof(float) * JET_DOF);
    memcpy(q_ext_a, dc_.jc_shm_->posExt, sizeof(float) * JET_DOF);

    q_ = Eigen::Map<Eigen::VectorQf>(q_a_, JET_DOF).cast<double>();
    q_virtual_local_.segment(7, JET_DOF) = q_;

    q_dot_ = Eigen::Map<Eigen::VectorQf>(q_dot_a_, JET_DOF).cast<double>();
    q_dot_virtual_local_.segment(6, JET_DOF) = q_dot_;
    q_ext_ = Eigen::Map<Eigen::VectorQf>(q_ext_a, JET_DOF).cast<double>();
    
    torque_ = Eigen::Map<Eigen::VectorQf>(torqueActual_a_, JET_DOF).cast<double>();

    if (dc_.jc_shm_->imuWriting)
        usleep(10);

    // getting from imu data (shared memory)
    q_virtual_local_(3) = dc_.jc_shm_->pos_virtual[3];
    q_virtual_local_(4) = dc_.jc_shm_->pos_virtual[4];
    q_virtual_local_(5) = dc_.jc_shm_->pos_virtual[5];
    q_virtual_local_(6) = dc_.jc_shm_->pos_virtual[6];

    q_dot_virtual_local_(3) = dc_.jc_shm_->vel_virtual[3];
    q_dot_virtual_local_(4) = dc_.jc_shm_->vel_virtual[4];
    q_dot_virtual_local_(5) = dc_.jc_shm_->vel_virtual[5];

    // memcpy(state_elmo_, dc_.jc_shm_->ecat_status, sizeof(int8_t) * JET_DOF);
    // memcpy(state_safety_, dc_.jc_shm_->safety_status, sizeof(int8_t) * JET_DOF);
    // memcpy(state_zp_, dc_.jc_shm_->zp_status, sizeof(int8_t) * JET_DOF);

}

void StateManager::UpdateYaw()
{
    q_virtual_local_yaw_initialized = q_virtual_local_;

    tf2::Quaternion q(q_virtual_local_(3), q_virtual_local_(4), q_virtual_local_(5), q_virtual_local_(6));
    tf2::Matrix3x3 m(q);
    m.getRPY(rd_local_.roll, rd_local_.pitch, rd_local_.yaw);

    if (dc_.inityawSwitch)
    {
        std::cout << " STATE : Yaw Initialized : " << rd_local_.yaw << std::endl;
        rd_.yaw_init = rd_local_.yaw;
        dc_.inityawSwitch = false;
    }

    tf2::Quaternion q_mod;
    rd_local_.yaw = rd_local_.yaw - rd_.yaw_init;

    q_mod.setRPY(rd_local_.roll, rd_local_.pitch, rd_local_.yaw);

    q_virtual_local_(3) = q_mod.getX();
    q_virtual_local_(4) = q_mod.getY();
    q_virtual_local_(5) = q_mod.getZ();
    q_virtual_local_(6) = q_mod.getW();
}

void StateManager::GetSensorData()
{
    rd_local_.imu_lin_acc(0) = dc_.jc_shm_->imu_acc[0];
    rd_local_.imu_lin_acc(1) = dc_.jc_shm_->imu_acc[1];
    rd_local_.imu_lin_acc(2) = dc_.jc_shm_->imu_acc[2];

    Eigen::Matrix3d pelv_imu_yaw;
    pelv_imu_yaw = DyrosMath::rotateWithZ(-rd_local_.yaw);
}

void StateManager::StateEstimate()
{
    if (rd_.semode && (!rd_.signal_yaw_init))
    {
        //////////////////Pos Estimate by IMU/////////////////////////////
        imu_lin_acc_lpf = DyrosMath::lpf(rd_.imu_lin_acc, imu_lin_acc_prev, 2000, 20);
        imu_lin_acc_prev = imu_lin_acc_lpf;
        // pelv_lin_acc = dc.link_[Pelvis].rotm.inverse() * imu_lin_acc_lpf;
        pelv_lin_acc = Eigen::Matrix3d::Matrix3d::Identity() * imu_lin_acc_lpf;

        // imu data recieving frequency is 2000Hz and need for calculating the base velocity and position
        double dt_i = 1.0 / 2000.0;
        Eigen::Vector3d temp;
        temp = imu_lin_vel_ + dt_i * pelv_lin_acc;
        imu_lin_vel_ = temp;
        temp = imu_pos_ + (dt_i * dt_i / 0.5) * pelv_lin_acc + imu_lin_vel_ * dt_i;
        imu_pos_ = temp;
        //////////////////Pos Estimate by IMU/////////////////////////////

        Vector3d imu_acc_dat;
        imu_acc_dat = link_local_[Pelvis].rotm * rd_local_.imu_lin_acc;

        imu_acc_dat = imu_acc_dat - imu_init;

        double dt = 0.0005;
        double tau = 0.4;
        double alpha = tau / (tau + dt);

        pelv_v = alpha * (imu_acc_dat * dt + pelv_v_before) + (1 - alpha) * mod_base_vel;
        pelv_v_before = pelv_v;
        q_virtual_ = q_virtual_local_;
        q_dot_virtual_ = q_dot_virtual_local_;


        pelv_x = alpha * (pelv_v * dt + imu_acc_dat * dt * dt * 0.5 + pelv_x_before) + (1 - alpha) * (-mod_base_pos);
        pelv_x_before = pelv_x;

        pelv_anga = (q_dot_virtual_.segment<3>(3) - rd_local_.imu_ang_vel_before) * 2000;
        rd_local_.imu_ang_vel_before = q_dot_virtual_.segment<3>(3);

        // mod_base_vel
        pelvis_velocity_estimate_ = pelv_v;
        static Vector3d base_vel_lpf = mod_base_vel;
        base_vel_lpf = DyrosMath::lpf(mod_base_vel, base_vel_lpf, 2000, 3);

        for (int i = 0; i < 3; i++)
        {
            q_virtual_(i) = imu_pos_(i);
            q_dot_virtual_(i) = imu_lin_vel_(i);
            // when using simulation true data
            // q_virtual_(i) = dc_.jc_shm_->pos_virtual[i];
            // q_dot_virtual_(i) = dc_.jc_shm_->vel_virtual[i];

            // q_ddot_virtual_(i) = rd_local_.imu_lin_acc(i); // dg test
            // q_ddot_virtual_(i + 3) = pelv_anga(i);
        }
    }
    else
    {
        q_virtual_ = q_virtual_local_;
        q_dot_virtual_ = q_dot_virtual_local_;
        // q_ddot_virtual_ = q_ddot_virtual_local_;
    }
}

void StateManager::StoreState(RobotData &rd_dst)
{
    memcpy(&rd_dst.model_, &model_global_, sizeof(RigidBodyDynamics::Model));

    // for (int i = 0; i < (LINK_NUMBER + 1); i++)
    // {
    //     memcpy(&rd_dst.link_[i].jac, &link_[i].jac, sizeof(Matrix6Vf));
    //     memcpy(&rd_dst.link_[i].jac_com, &link_[i].jac_com, sizeof(Matrix6Vf));

    //     memcpy(&rd_dst.link_[i].xpos, &link_[i].xpos, sizeof(Vector3d));
    //     memcpy(&rd_dst.link_[i].xipos, &link_[i].xipos, sizeof(Vector3d));
    //     memcpy(&rd_dst.link_[i].rotm, &link_[i].rotm, sizeof(Matrix3d));
    //     memcpy(&rd_dst.link_[i].v, &link_[i].v, sizeof(Vector3d));
    //     memcpy(&rd_dst.link_[i].vi, &link_[i].vi, sizeof(Vector3d));
    //     memcpy(&rd_dst.link_[i].w, &link_[i].w, sizeof(Vector3d));

    //     // xpos xipos rotm v w
    // }

    // memcpy(&rd_dst.A_, &A_, sizeof(MatrixVVd));
    // memcpy(&rd_dst.A_inv_, &A_inv_, sizeof(MatrixVVd));
    // memcpy(&rd_dst.Motor_inertia, &Motor_inertia, sizeof(MatrixVVd));
    // memcpy(&rd_dst.Motor_inertia_inverse, &Motor_inertia_inverse, sizeof(MatrixVVd));
    memcpy(&rd_dst.q_, &q_, sizeof(VectorQd));
    memcpy(&rd_dst.q_dot_, &q_dot_, sizeof(VectorQd));
    memcpy(&rd_dst.q_ext_, &q_ext_, sizeof(VectorQd));
    memcpy(&rd_dst.q_virtual_, &q_virtual_, sizeof(VectorQVQd));
    memcpy(&rd_dst.q_dot_virtual_, &q_dot_virtual_, sizeof(VectorVQd));
    memcpy(&rd_dst.q_ddot_virtual_, &q_ddot_virtual_, sizeof(VectorVQd));
    memcpy(&rd_dst.torque_, &torque_, sizeof(VectorQd));

    rd_dst.roll = rd_local_.roll;
    rd_dst.pitch = rd_local_.pitch;
    rd_dst.yaw = rd_local_.yaw;

    // if (!rd_dst.firstCalc)
    // {

    //     memcpy(&rd_dst.link_, link_, (LINK_NUMBER + 1) * sizeof(LinkData));

    //     rd_dst.firstCalc = true;
    // }

    rd_dst.control_time_ = control_time_;
    rd_dst.tp_state_ = rd_local_.tp_state_;

    dc_.triggerThread1 = true;
}

void StateManager::SendCommand()
{
    timespec t_u10;

    t_u10.tv_nsec = 10000;
    t_u10.tv_sec = 0;
    static double joint_command[JET_DOF];
    while (dc_.t_c_)
    {
        clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    }
    dc_.t_c_ = true;
    std::copy(dc_.joint_command, dc_.joint_command + JET_DOF, joint_command);
    static int rcv_c_count = dc_.control_command_count;

    dc_.t_c_ = false;

    static int rcv_c_count_before;
    static int warning_cnt = 0;
    if (rcv_c_count_before == rcv_c_count)
    {
        warning_cnt++;
    }

    if (warning_cnt > 0)
    {
        static int prob_cnt;
        prob_cnt = rcv_c_count;

        if (warning_cnt > 10)
        {
            if (prob_cnt != rcv_c_count)
            {
                std::cout << " STATE : Command not received for " << warning_cnt << "times " << std::endl;
                warning_cnt = 0;
            }
        }
        else if (prob_cnt != rcv_c_count)
        {
            warning_cnt = 0;
        }
    }

    rcv_c_count_before = rcv_c_count;

    const double maxTorque = _MAXTORQUE; // SYSTEM MAX TORQUE

    const double rTime1 = 4.0;
    const double rTime2 = 1.0;

    const double rat1 = 0.3;
    const double rat2 = 0.7;

    int maxTorqueCommand;

    float control_time_at_ = rd_gl_.control_time_;

    if (dc_.torqueOnSwitch)
    {
        dc_.rd_.positionControlSwitch = true;

        dc_.torqueOnSwitch = false;

        if (dc_.torqueOn)
        {
            std::cout << " STATE : Torque is already on " << std::endl;
        }
        else
        {
            std::cout << " STATE : Turning on ... " << std::endl;
            dc_.torqueOnTime = control_time_at_;
            dc_.torqueOn = true;
            dc_.torqueRisingSeq = true;

        }
    }
    if (dc_.torqueOffSwitch)
    {
        dc_.torqueOffSwitch = false;

        if (dc_.torqueOn)
        {
            std::cout << " STATE : Turning off ... " << std::endl;
            dc_.torqueOffTime = control_time_at_;
            dc_.toruqeDecreaseSeq = true;
        }
        else
        {
            std::cout << " STATE : Torque is already off" << std::endl;
        }
    }

    if (dc_.torqueOn)
    {
        if (dc_.torqueRisingSeq)
        {
            if (control_time_at_ <= dc_.torqueOnTime + rTime1)
            {
                torqueRatio = rat1 * DyrosMath::minmax_cut((control_time_at_ - dc_.torqueOnTime) / rTime1, 0.0, 1.0);
            }
            if (control_time_at_ > dc_.torqueOnTime + rTime1 && control_time_at_ <= dc_.torqueOnTime + rTime1 + rTime2)
            {
                torqueRatio = rat1 + rat2 * DyrosMath::minmax_cut((control_time_at_ - dc_.torqueOnTime - rTime1) / rTime2, 0.0, 1.0);
            }
            else if (control_time_at_ > dc_.torqueOnTime + rTime1 + rTime2)
            {
                std::cout << " STATE : Torque 100% ! " << std::endl;
                StatusPub("%f Torque 100%", control_time_);

                torqueRatio = 1.0;

                dc_.torqueRisingSeq = false;
            }

            maxTorqueCommand = maxTorque * torqueRatio;
        }
        else if (dc_.toruqeDecreaseSeq)
        {

            if (control_time_at_ <= dc_.torqueOffTime + rTime2)
            {
                torqueRatio = (1 - rat2 * DyrosMath::minmax_cut((control_time_at_ - dc_.torqueOffTime) / rTime2, 0.0, 1.0));
            }
            if (control_time_at_ > dc_.torqueOffTime + rTime2 && control_time_at_ <= dc_.torqueOffTime + rTime2 + rTime1)
            {
                torqueRatio = (1 - rat2 - rat1 * DyrosMath::minmax_cut((control_time_at_ - dc_.torqueOffTime - rTime2) / rTime1, 0.0, 1.0));
            }
            else if (control_time_at_ > dc_.torqueOffTime + rTime2 + rTime1)
            {
                dc_.toruqeDecreaseSeq = false;

                rd_gl_.tc_run = false;

                std::cout << " STATE : Torque 0% .. torque Off " << std::endl;
                StatusPub("%f Torque 0%", control_time_);
                torqueRatio = 0.0;
                dc_.torqueOn = false;
            }

            maxTorqueCommand = maxTorque * torqueRatio;
        }
        else
        {
            torqueRatio = 1.0;
            maxTorqueCommand = (int)maxTorque;
        }
    }
    else
    {
        torqueRatio = 0.0;
        maxTorqueCommand = 0;
    }

    if (dc_.emergencySwitch)
    {
        dc_.emergencyStatus = true; //
        rd_gl_.tc_run = false;
        rd_gl_.pc_mode = false;
    }

    if (dc_.emergencyStatus)
    {
        for (int i = 0; i < JET_DOF; i++)
            joint_command[i] = 0.0;
    }

    dc_.jc_shm_->commanding = true;

    // // UpperBody
    // while (dc_.jc_shm_->cmd_upper)
    // {
    //     clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    // }
    // dc_.jc_shm_->cmd_upper = true;
    // std::copy(torque_command + 15, torque_command + JET_DOF, dc_.jc_shm_->torqueCommand + 15);
    // dc_.jc_shm_->maxTorque = maxTorqueCommand;
    // cCount++;
    // dc_.jc_shm_->commandCount.store(cCount);

    // dc_.jc_shm_->cmd_upper = false;

    // // LowerBody
    // while (dc_.jc_shm_->cmd_lower)
    // {
    //     clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
    // }
    // dc_.jc_shm_->cmd_lower = true;
    // std::copy(joint_command, joint_command + 15, dc_.jc_shm_->torqueCommand);
    // dc_.jc_shm_->cmd_lower = false;

    dc_.jc_shm_->commanding.store(false);
}

void StateManager::SimCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::msg::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_sim_command_pub_->publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::msg::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_sim_command_pub_->publish(rst_msg_);

        control_time_ = 0.0;
    }

    if (buf == "terminate")
    {
        dc_.jc_shm_->shutdown = true;
    }
}