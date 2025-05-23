#ifndef LITE3_HARDWARE_INTERFACE_HPP_
#define LITE3_HARDWARE_INTERFACE_HPP_

#include "robot_interface.h"
#include "ehr_hardware.h"
#include <memory>

class Lite3HardwareInterface : public RobotInterface {
private:
    std::shared_ptr<EhrHardware> ehr_hw_;
    ehr_body_state state_;
    Vec3f omega_body_, rpy_, acc_;
    VecXf joint_pos_, joint_vel_, joint_tau_;

public:
    Lite3HardwareInterface(const std::string& robot_name)
    : RobotInterface(robot_name, 12) {
        std::cout << robot_name << " is using EHR Hardware Interface" << std::endl;

        ehr_hw_ = std::make_shared<EhrHardware>();
        ehr_hw_->GetInterface()->Init("/home/ehr/wxy/eqr_rl_deploy/interface/robot/hardware/hardware_interface_ehr02.yaml");

        state_.joint_num = dof_num_;
        state_.q_joints = new double[dof_num_];
        state_.v_joints = new double[dof_num_];
        state_.f_joints = new double[dof_num_];
        state_.gear_ratios = new double[dof_num_];

        state_.kp_joints = new double[dof_num_];
        state_.kd_joints = new double[dof_num_];
        state_.ki_joints = new double[dof_num_];

        state_.q_joint_desired = new double[dof_num_];
        state_.v_joint_desired = new double[dof_num_];
        state_.ff_joint_desired = new double[dof_num_];
    }

    ~Lite3HardwareInterface() {
        delete[] state_.q_joints;
        delete[] state_.v_joints;
        delete[] state_.f_joints;
        delete[] state_.gear_ratios;
        delete[] state_.kp_joints;
        delete[] state_.kd_joints;
        delete[] state_.ki_joints;
        delete[] state_.q_joint_desired;
        delete[] state_.v_joint_desired;
        delete[] state_.ff_joint_desired;
    }

    virtual void Start() override {
        ehr_hw_->GetInterface()->Read(&state_);
    }

    virtual void Stop() override {
        // 暂无特定停止控制逻辑，如有可在此添加
    }

    virtual double GetInterfaceTimeStamp() override {
        // ⚠️ 无法完善：state_ 结构中没有明确的时间戳字段（如 tick 或 timestamp）
        // 请确认是否存在某字段记录采样时间，单位是否为 ms
        return 0.0;
    }

    virtual VecXf GetJointPosition() override {
        joint_pos_ = VecXf::Zero(dof_num_);
        for (int i = 0; i < dof_num_; ++i)
            joint_pos_(i) = state_.q_joints[i];
        return joint_pos_;
    }

    virtual VecXf GetJointVelocity() override {
        joint_vel_ = VecXf::Zero(dof_num_);
        for (int i = 0; i < dof_num_; ++i)
            joint_vel_(i) = state_.v_joints[i];
        return joint_vel_;
    }

    virtual VecXf GetJointTorque() override {
        joint_tau_ = VecXf::Zero(dof_num_);
        for (int i = 0; i < dof_num_; ++i)
            joint_tau_(i) = state_.f_joints[i];
        return joint_tau_;
    }

    virtual Vec3f GetImuRpy() override {
        // ⚠️ 假设 q2_base[3~5] 单位为角度（°）
        rpy_ << state_.q2_base[3] * M_PI / 180.0,
                state_.q2_base[4] * M_PI / 180.0,
                state_.q2_base[5] * M_PI / 180.0;
        return rpy_;
    }

    virtual Vec3f GetImuAcc() override {
        acc_ << state_.a_base[0], state_.a_base[1], state_.a_base[2];
        return acc_;
    }

    virtual Vec3f GetImuOmega() override {
        omega_body_ << state_.v_base[3], state_.v_base[4], state_.v_base[5];
        return omega_body_;
    }

    virtual VecXf GetContactForce() override {
        // ⚠️ 无法完善：state_ 结构体未提供足端受力数据，暂设为0
        return VecXf::Zero(4);
    }

    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) override {
        for (int i = 0; i < dof_num_; ++i) {
            state_.kp_joints[i]        = input(i, 0);
            state_.q_joint_desired[i]  = input(i, 1);
            state_.kd_joints[i]        = input(i, 2);
            state_.v_joint_desired[i]  = input(i, 3);
            state_.ff_joint_desired[i] = input(i, 4);
        }
        ehr_hw_->GetInterface()->Write(state_);
        ehr_hw_->GetInterface()->Read(&state_);
    }
};

#endif  // EHR_HARDWARE_INTERFACE_HPP_
