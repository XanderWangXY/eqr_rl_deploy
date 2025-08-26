/**
 * @file lite3_test_policy_runner.hpp
 * @brief 
 * @author mazunwang
 * @version 1.0
 * @date 2024-06-07
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef GO2_POLICY_RUNNER_TS_HPP_
#define GO2_POLICY_RUNNER_TS_HPP_

#include "policy_runner_base.hpp"

class Go2PolicyRunner : public PolicyRunnerBase
{
private:
    std::string policy_path_;

    torch::jit::Module backbone_;
    torch::Tensor action_tensor_, obs_total_tensor_;
    std::vector<c10::IValue> obs_vector_{};

    int obs_dim_ = 45;
    int obs_his_num_ = 50;
    int act_dim_ = 12;
    int obs_total_dim_;

    VecXf current_obs_, obs_history_, obs_total_;
    VecXf last_dof_pos0_, last_dof_pos1_, last_dof_pos2_;
    VecXf last_dof_vel0_, last_dof_vel1_;
    VecXf last_action1_, last_action0_, action_,last_action;

    VecXf action_scale;

    VecXf dof_pos_default_;
    VecXf kp_, kd_;
    Vec3f max_cmd_vel_;

    // ✅ 新增：实现计算投影重力的函数
    Vec3f CalculateProjectedGravity(const Vec3f& rpy) {
        float roll = rpy[0];   // Access roll
        float pitch = rpy[1];  // Access pitch
        float yaw = rpy[2];    // Access yaw

        // 计算四元数分量
        float w = std::cos(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
        float x = std::sin(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) - std::cos(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
        float y = std::cos(roll/2) * std::sin(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::cos(pitch/2) * std::sin(yaw/2);
        float z = std::cos(roll/2) * std::cos(pitch/2) * std::sin(yaw/2) - std::sin(roll/2) * std::sin(pitch/2) * std::cos(yaw/2);

        // 向量v = [0, 0, -1]
        const float v_x = 0.0f;
        const float v_y = 0.0f;
        const float v_z = -1.0f;

        // 计算a = v * (2 * q_w² - 1)
        float scalar = 2.0f * w * w - 1.0f;
        float a_x = v_x * scalar;
        float a_y = v_y * scalar;
        float a_z = v_z * scalar;

        // 计算b = cross(q_vec, v) * q_w * 2.0
        float b_x = (y * v_z - z * v_y) * w * 2.0f; // 叉乘(q_vec x v)
        float b_y = (z * v_x - x * v_z) * w * 2.0f;
        float b_z = (x * v_y - y * v_x) * w * 2.0f;

        // 计算c = q_vec * dot(q_vec, v) * 2.0
        float dot_product = x * v_x + y * v_y + z * v_z;
        float c_x = x * dot_product * 2.0f;
        float c_y = y * dot_product * 2.0f;
        float c_z = z * dot_product * 2.0f;

        // 最终结果 = a - b + c
        return Vec3f(a_x - b_x + c_x, a_y - b_y + c_y, a_z - b_z + c_z);
    }

public:
    Go2PolicyRunner(std::string policy_name):PolicyRunnerBase(policy_name){
        policy_path_ = GetAbsPath()+"/../policy/go2_amp_rough.pt";
        obs_total_dim_ = obs_dim_ + obs_his_num_*obs_dim_;
        dof_pos_default_.setZero(12);
        dof_pos_default_ << 0.1, 0.8, -1.5,
                            -0.1, 0.8, -1.5,
                            0.1, 1.0, -1.5,
                            -0.1, 1.0, -1.5;
        kp_ = 25.*VecXf::Ones(12);
        kd_ = 0.5*VecXf::Ones(12);
        max_cmd_vel_ << 2.0, 0.5, 1.57;

        // ✅ 初始化缩放系数：第1,4,7,10位（索引0,3,6,9）为0.125，其余为0.25
        // action_scale.resize(12);  // 12个电机对应12个系数
        // for(int i=0; i<12; ++i){
        //     if(i % 3 == 0) {  // 索引0,3,6,9（每3个一组的第一个）
        //         action_scale(i) = 0.125;
        //     } else {
        //         action_scale(i) = 0.25;
        //     }
        // }

        try { 
            backbone_ = torch::jit::load(policy_path_); 
            backbone_.eval();
        }
        catch (const c10::Error &e) { std::cerr << "error loading policy at " << policy_path_ << "\n" << e.what(); }

        for(int i=0;i<10;++i){
            torch::Tensor reaction;
            obs_vector_.clear();
            obs_vector_.emplace_back(torch::ones({1, obs_total_dim_}));  
            reaction = backbone_.forward(obs_vector_).toTensor();
            if(i==9) std::cout << policy_name_ << " network test success" << std::endl;
        }

        decimation_ = 1;
    }
    ~Go2PolicyRunner(){
    }

    void DisplayPolicyInfo(){
        std::cout << "name : " << policy_name_ << std::endl; 
        std::cout << "path : " << policy_path_ << std::endl;
        std::cout << "dim  : " << obs_dim_ << " " << obs_his_num_ << " " << obs_total_dim_ << " " << act_dim_ << std::endl;
        std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
        std::cout << "kp   : " << kp_.transpose() << std::endl;
        std::cout << "kd   : " << kd_.transpose() << std::endl;
        std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
    }
    

    void  OnEnter(){
        run_cnt_ = 0;
        current_obs_.setZero(obs_dim_);
        obs_history_.setZero(obs_dim_*obs_his_num_);
        obs_total_.setZero(obs_total_dim_); 
        std::cout << "enter " << policy_name_ << std::endl;
    }

    RobotAction GetRobotAction(const RobotBasicState& ro){
        Vec3f cmd_vel = ro.cmd_vel_normlized.cwiseProduct(max_cmd_vel_);

        if(run_cnt_ == 0){
            last_dof_pos2_ = last_dof_pos1_ = last_dof_pos0_ = ro.joint_pos;
            last_dof_vel1_ = last_dof_vel0_ = ro.joint_vel;
            last_action1_ = last_action0_ = ro.joint_pos;
            last_action0_.setZero(ro.joint_pos.size());
        }

        // Vec3f projected_gravity = CalculateProjectedGravity(ro.base_rpy);
        
        // ✅ 新增：定义新顺序索引
        const std::vector<int> new_order = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};
        
        // ✅ 重新排列 ro.joint_pos - dof_pos_default_
        VecXf joint_pos_diff_reordered(12);
        for(int i = 0; i < 12; ++i) {
            int original_idx = new_order[i];
            joint_pos_diff_reordered[i] = (ro.joint_pos)[original_idx];
        }

        // ✅ 重新排列 0.05*ro.joint_vel
        VecXf joint_vel_reordered(12);
        for(int i = 0; i < 12; ++i) {
            int original_idx = new_order[i];
            joint_vel_reordered[i] = (0.1 * ro.joint_vel)[original_idx];
        }

        current_obs_.setZero(obs_dim_);
        current_obs_ << cmd_vel,
                        ro.base_rpy,
                        ro.base_omega,
                        joint_pos_diff_reordered - dof_pos_default_,
                        joint_vel_reordered,
                        // last_dof_pos2_, 
                        // last_dof_pos1_,
                        // last_dof_pos0_,  // 72
                        // 0.1*last_dof_vel1_,
                        // 0.1*last_dof_vel0_,
                        // last_action1_,
                        last_action0_;

        // std::cout << "current_obs_ :" << current_obs_ << std::endl;

        // std::cout << "cmd_vel :" << cmd_vel.transpose() << std::endl;
        // std::cout << "projected_gravity :" << projected_gravity << std::endl;
        // std::cout << "0.25*ro.base_omega :" << 0.25*ro.base_omega.transpose() << std::endl;
        // std::cout << "ro.joint_pos - dof_pos_default_ :" << (ro.joint_pos - dof_pos_default_).transpose() << std::endl;
        // std::cout << "0.05*ro.joint_vel :" << (0.05*ro.joint_vel).transpose() << std::endl;
        // std::cout << "last_action0_ :" << last_action0_.transpose() << std::endl;
        //std::cout << "ro.joint_pos:" << (ro.joint_pos).transpose() << std::endl;

        VecXf obs_history_record = obs_history_.segment(obs_dim_, (obs_his_num_-1)*obs_dim_).eval();
        obs_history_.segment(0, (obs_his_num_-1)*obs_dim_) = obs_history_record;
        obs_history_.segment((obs_his_num_-1)*obs_dim_, obs_dim_) = current_obs_;

        obs_total_.segment(0, obs_dim_) = current_obs_;
        obs_total_.segment(obs_dim_, obs_dim_*obs_his_num_) = obs_history_;

        last_dof_pos2_ = last_dof_pos1_;
        last_dof_pos1_ = last_dof_pos0_;
        last_dof_pos0_ = ro.joint_pos;

        last_dof_vel1_ = last_dof_vel0_;
        last_dof_vel0_ = ro.joint_vel;

        Eigen::MatrixXf temp = obs_total_.transpose();
        torch::Tensor a = torch::from_blob(temp.data(), {temp.rows(), temp.cols()}, torch::kFloat);
        obs_total_tensor_ = a.clone();

        obs_vector_.clear();
        obs_vector_.emplace_back(obs_total_tensor_);
        // std::cout << "obs_vector_ :" << obs_vector_ << std::endl;
        action_tensor_ = backbone_.forward(obs_vector_).toTensor();

        // Eigen::Matrix<float, 12, 1> act(action_tensor_.data_ptr<float>());
        Eigen::Map<Eigen::MatrixXf> act(action_tensor_.data_ptr<float>(), act_dim_, 1);
        // last_action = last_action1_;
        // last_action1_ = last_action0_;
        // last_action0_ = act.col(0);
        // last_action = last_action0_;
        // std::cout << "action_ :" << action_.transpose() << std::endl;

        // action_ = act.col(0).cwiseProduct(action_scale);
        action_ = 0.25*act.col(0);

        last_action = last_action1_;
        last_action1_ = last_action0_;
        last_action0_ = action_*4;
        // std::cout << "action_ :" << action_.transpose() << std::endl;
        // std::cout << "action_scale :" << action_scale.transpose() << std::endl;
        RobotAction ra;
        ra.goal_joint_pos = action_ + dof_pos_default_;

        // ra.goal_joint_pos.setZero(12);
        // // 将第一个元素设置为 1
        // ra.goal_joint_pos[5] = -2;
        // ra.goal_joint_pos = ra.goal_joint_pos*0.85 + 0.15 * (last_action*0.25+dof_pos_default_);
        // ✅ 新增：按新顺序调整关节位置
        
        VecXf reordered_pos(12); // 存储重新排列后的结果
        for(int i = 0; i < 12; ++i) {
            int original_idx = new_order[i]; // 原索引
            reordered_pos[i] = ra.goal_joint_pos[original_idx]; // 按新顺序提取元素
        }
        ra.goal_joint_pos = reordered_pos; // 更新为新顺序的关节位置
        
        ra.goal_joint_vel = VecXf::Zero(act_dim_);
        ra.tau_ff = VecXf::Zero(act_dim_);
        ra.kp = kp_;
        ra.kd = kd_;
        ++run_cnt_;

        return ra;
    }
};

#endif

