#ifndef LITE3_HARDWARE_INTERFACE_HPP_
#define LITE3_HARDWARE_INTERFACE_HPP_

#include "robot_interface.h"
#include "ehr_hardware.h"
#include <memory>
#include <Eigen/Geometry>

Vec3f ConvertNEDQuaternionToRPY_ENU(double q_w, double q_x, double q_y, double q_z) {
    // 原始 NED 四元数 → 转换为 Eigen 四元数
    Eigen::Quaterniond quat_ned(q_w, q_x, q_y, q_z);
    
    // 转为旋转矩阵
    Eigen::Matrix3d R_ned = quat_ned.toRotationMatrix();

    // 坐标系转换矩阵：NED → ENU，相当于反转 Y, Z
    Eigen::Matrix3d R_convert = Eigen::Matrix3d::Identity();
    R_convert(1,1) = -1;  // Y
    R_convert(2,2) = -1;  // Z

    // 应用坐标变换
    Eigen::Matrix3d R_enu = R_convert * R_ned * R_convert;

    // 从旋转矩阵提取 RPY（ZYX，即 Yaw-Pitch-Roll）
    Eigen::Vector3d rpy = R_enu.eulerAngles(2, 1, 0);  // Yaw-Pitch-Roll

    return Vec3f(rpy[2], rpy[1], rpy[0]); // Roll, Pitch, Yaw
}


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
        std::string yaml_file = "/home/eqr2/eqr_rl_deploy/interface/robot/hardware/hardware_interface_ehr02.yaml";
        ehr_hw_ = std::make_shared<EhrHardware>();
        std::cout << "=====yaml file address:" << yaml_file.c_str() << " len" << yaml_file.length() << std::endl;
        ehr_hw_->GetInterface()->Init(yaml_file.c_str(), yaml_file.length());

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
    return ConvertNEDQuaternionToRPY_ENU(
        state_.q_base[3],  // w
        state_.q_base[4],  // x
        state_.q_base[5],  // y
        state_.q_base[6]   // z
    );
    }

    virtual Vec3f GetImuAcc() override {
        acc_ << state_.a_base[0], state_.a_base[1], state_.a_base[2];
        return acc_;
    }

    virtual Vec3f GetImuOmega() override {
    // 原始角速度（NED系）：[roll_rate, pitch_rate, yaw_rate]
    double wx_ned = state_.v_base[3];
    double wy_ned = state_.v_base[4];
    double wz_ned = state_.v_base[5];

    // 坐标变换：NED → ENU（Y/Z 取负）
    omega_body_ << wx_ned, -wy_ned, -wz_ned;
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
