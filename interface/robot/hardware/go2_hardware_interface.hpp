#ifndef GO2_HARDWARE_INTERFACE_HPP_
#define GO2_HARDWARE_INTERFACE_HPP_

#include "robot_interface.h"
#include "common_types.h"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/common/thread/thread.hpp>

#include <atomic>
#include <iostream>

using namespace unitree::robot;
using namespace unitree::common;
using namespace unitree_go;         // IDL 命名空间
using namespace interface;         // RobotInterface
using namespace types;             // Vec3f/VecXf/MatXf

class Go2HardwareInterface : public RobotInterface
{
private:
    ChannelPublisherPtr<msg::dds_::LowCmd_>    lowcmd_pub_;
    ChannelSubscriberPtr<msg::dds_::LowState_> lowstate_sub_;
    ThreadPtr                                  write_thread_;

    msg::dds_::LowState_ low_state_{};
    msg::dds_::LowCmd_   low_cmd_{};

    Vec3f omega_body_, rpy_, acc_;
    VecXf joint_pos_, joint_vel_, joint_tau_;

    double            timestamp_{0.0};
    const float       dt_{0.002f};
    std::atomic<bool> running_{false};

public:
    explicit Go2HardwareInterface(const std::string& robot_name,
                                  const std::string& net_if = "eth0")
        : RobotInterface(robot_name, 12)
    {
        std::cout << robot_name << " is using Go2 Hardware Interface\n";

        ChannelFactory::Instance()->Init(0, net_if.c_str());

        lowcmd_pub_.reset(new ChannelPublisher<msg::dds_::LowCmd_>("rt/lowcmd"));
        lowcmd_pub_->InitChannel();

        lowstate_sub_.reset(new ChannelSubscriber<msg::dds_::LowState_>("rt/lowstate"));
        lowstate_sub_->InitChannel(
            std::bind(&Go2HardwareInterface::LowStateCallback, this, std::placeholders::_1), 1);

        InitLowCmd();
    }

    ~Go2HardwareInterface() override
    {
        running_ = false;
        write_thread_.reset();   // ✔️ Thread::StopWork() 是 SDK 提供
    }

    /* --------------------- 基类实现 --------------------- */
    void Start() override
    {
        running_ = true;
        write_thread_ = CreateRecurrentThreadEx(
            "go2_lowcmd_writer", UT_CPU_ID_NONE,
            static_cast<int>(dt_ * 1e6),
            &Go2HardwareInterface::LowCmdWriter, this);
    }

    void Stop() override
    {
        running_ = false;
        write_thread_.reset();   // ✔️
    }

    double GetInterfaceTimeStamp() override { return timestamp_; }

    VecXf GetJointPosition() override
    {
        joint_pos_ = VecXf::Zero(dof_num_);
        for (int i = 0; i < dof_num_; ++i)
            joint_pos_(i) = low_state_.motor_state()[i].q();
        return joint_pos_;
    }

    VecXf GetJointVelocity() override
    {
        joint_vel_ = VecXf::Zero(dof_num_);
        for (int i = 0; i < dof_num_; ++i)
            joint_vel_(i) = low_state_.motor_state()[i].dq();
        return joint_vel_;
    }

    VecXf GetJointTorque() override
    {
        joint_tau_ = VecXf::Zero(dof_num_);
        for (int i = 0; i < dof_num_; ++i)
            joint_tau_(i) = low_state_.motor_state()[i].tau_est();   // <- 若叫 tau() 或 tauEst() 改这里
        return joint_tau_;
    }

    Vec3f GetImuRpy() override
    {
        rpy_ << low_state_.imu_state().rpy()[0],
                low_state_.imu_state().rpy()[1],
                low_state_.imu_state().rpy()[2];
        return rpy_;
    }

    Vec3f GetImuAcc() override
    {
        acc_ << low_state_.imu_state().accelerometer()[0],         // <- 若无 acc()，改为 acc_x()/acc_y()/acc_z()
                low_state_.imu_state().accelerometer()[1],
                low_state_.imu_state().accelerometer()[2];
        return acc_;
    }

    Vec3f GetImuOmega() override
    {
        omega_body_ << low_state_.imu_state().gyroscope()[0],  // <- 若无 gyr()，改为 gyr_x()/gyr_y()/gyr_z()
                       low_state_.imu_state().gyroscope()[1],
                       low_state_.imu_state().gyroscope()[2];
        return omega_body_;
    }

    VecXf GetContactForce() override { return VecXf::Zero(4); }

    void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) override
    {
        joint_cmd_ = input;
        for (int i = 0; i < dof_num_; ++i) {
            low_cmd_.motor_cmd()[i].kp()  = input(i, 0);
            low_cmd_.motor_cmd()[i].q()   = input(i, 1);
            low_cmd_.motor_cmd()[i].kd()  = input(i, 2);
            low_cmd_.motor_cmd()[i].dq()  = input(i, 3);
            low_cmd_.motor_cmd()[i].tau() = input(i, 4);
        }
        low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_),
                                    (sizeof(msg::dds_::LowCmd_) >> 2) - 1);
    }

private:
    /* -------------------- SDK 回调 -------------------- */
    void LowStateCallback(const void* msg)
    {
        low_state_ = *static_cast<const msg::dds_::LowState_*>(msg);
        timestamp_ = low_state_.tick() * 1e-3;          // tick = 1 ms
    }

    /* -------------------- 写线程 -------------------- */
    void LowCmdWriter()
    {
        if (!running_) return;
        lowcmd_pub_->Write(low_cmd_);
    }

    /* -------------------- 初始化 -------------------- */
    void InitLowCmd()
    {
        constexpr float PosStopF = 2.146E+9f;
        constexpr float VelStopF = 16000.0f;

        low_cmd_.head()[0]   = 0xFE;
        low_cmd_.head()[1]   = 0xEF;
        low_cmd_.level_flag() = 0xFF;
        low_cmd_.gpio()       = 0;

        for (int i = 0; i < 20; ++i) {
            low_cmd_.motor_cmd()[i].mode() = 0x01;
            low_cmd_.motor_cmd()[i].q()    = PosStopF;
            low_cmd_.motor_cmd()[i].kp()   = 0;
            low_cmd_.motor_cmd()[i].dq()   = VelStopF;
            low_cmd_.motor_cmd()[i].kd()   = 0;
            low_cmd_.motor_cmd()[i].tau()  = 0;
        }
    }

    /* ------------------- CRC32 ------------------- */
    static uint32_t crc32_core(uint32_t* ptr, uint32_t len)
    {
        const uint32_t poly = 0x04C11DB7;
        uint32_t crc = 0xFFFFFFFF;
        for (uint32_t i = 0; i < len; ++i) {
            uint32_t data = ptr[i];
            for (int b = 0; b < 32; ++b) {
                bool msb = crc & 0x80000000;
                crc <<= 1;
                if (data & 0x80000000) crc ^= poly;
                if (msb)               crc ^= poly;
                data <<= 1;
            }
        }
        return crc;
    }
};

#endif  // GO2_HARDWARE_INTERFACE_HPP_
