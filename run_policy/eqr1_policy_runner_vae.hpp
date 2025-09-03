#ifndef EQR1_POLICY_RUNNER_VAE_HPP_
#define EQR1_POLICY_RUNNER_VAE_HPP_

#include "policy_runner_base.hpp"

class Eqr1PolicyRunner : public PolicyRunnerBase 
{
private:
    std::string policy_path_;

    torch::jit::Module backbone_;
    torch::Tensor action_tensor_, obs_total_tensor_;
    torch::Tensor obs_vector_;
    c10::IValue test;

    int obs_dim_ = 45;
    int obs_his_num_ = 10;
    int act_dim_ = 12;
    int env_num = 1;

    VecXf dof_pos_default_;
    VecXf kp_, kd_;
    Vec3f max_cmd_vel_;
    VecXf last_action;
    VecXf action_;
    VecXf new_action_;

    torch::Tensor his_obs = torch::zeros({env_num, obs_his_num_, obs_dim_}, torch::kFloat32);
    torch::Tensor cur_obs = torch::zeros({env_num, 1, 45}, torch::kFloat32);
    torch::Tensor obs_history_tensor_ = torch::zeros({1, 10, 45}, torch::kFloat32);

public:
    Eqr1PolicyRunner(std::string policy_name):PolicyRunnerBase(policy_name){
        policy_path_ = GetAbsPath()+"/../policy/eqr1_test.pt";
        dof_pos_default_.setZero(12);
        dof_pos_default_ << 0.0, 0.0, 0.0, 0.0,
                            -0.5, -0.5, -0.5, -0.5,
                            1.1, 1.1, 1.1, 1.1;
        kp_ = 25.*VecXf::Ones(12);
        kd_ = 0.5*VecXf::Ones(12);
        max_cmd_vel_ << 1.0, 1.0, 1.0;

        try { 
            backbone_ = torch::jit::load(policy_path_); 
            backbone_.to(torch::kCPU);
            backbone_.eval();
        }
        catch (const c10::Error &e) { std::cerr << "error loading policy at " << policy_path_ << "\n" << e.what(); }

        for(int i=0;i<10;++i){
            std::vector<c10::IValue> inputs;
            inputs.push_back(c10::IValue(obs_history_tensor_));

            test = backbone_.forward(inputs);
            if(i==9) std::cout << policy_name_ << " network test success" << std::endl;
        }
        decimation_ = 2;
    }
    

    ~Eqr1PolicyRunner(){
    }

    void DisplayPolicyInfo(){
        std::cout << "name : " << policy_name_ << std::endl; 
        std::cout << "path : " << policy_path_ << std::endl;
        std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
        std::cout << "kp   : " << kp_.transpose() << std::endl;
        std::cout << "kd   : " << kd_.transpose() << std::endl;
        std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
    }
    
    
    void  OnEnter(){
        run_cnt_ = 0;
        std::cout << "enter " << policy_name_ << std::endl;
        last_action = dof_pos_default_;
    }

    RobotAction GetRobotAction(const RobotBasicState& ro){
        
        Vec3f cmd_vel = ro.cmd_vel_normlized.cwiseProduct(max_cmd_vel_);


        // added by yxc for the projected gravity.
        float roll = ro.base_rpy[0];  // Access roll
        float pitch = ro.base_rpy[1]; // Access pitch
        float yaw = ro.base_rpy[2];   // Access yaw
        // Calculate quaternion components
        float w = std::cos(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
        float x = std::sin(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) - std::cos(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
        float y = std::cos(roll/2) * std::sin(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::cos(pitch/2) * std::sin(yaw/2);
        float z = std::cos(roll/2) * std::cos(pitch/2) * std::sin(yaw/2) - std::sin(roll/2) * std::sin(pitch/2) * std::cos(yaw/2);
        // Since v is always [0, 0, -1], we define it explicitly:
        float v_x = 0.0f;
        float v_y = 0.0f;
        float v_z = -1.0f;
        // Compute a = v * (2 * q_w^2 - 1)
        float scalar = 2.0f * w * w - 1.0f;
        float a_x = v_x * scalar;
        float a_y = v_y * scalar;
        float a_z = v_z * scalar;
        // Compute b = cross(q_vec, v) * q_w * 2.0
        float b_x = y * v_z - z * v_y; // Cross product (q_vec x v)
        float b_y = z * v_x - x * v_z;
        float b_z = x * v_y - y * v_x;
        b_x *= w * 2.0f;
        b_y *= w * 2.0f;
        b_z *= w * 2.0f;
        // Compute c = q_vec * dot(q_vec, v) * 2.0
        // Dot product q_vec . v = q_x * v_x + q_y * v_y + q_z * v_z
        float dot_product = x * v_x + y * v_y + z * v_z;
        float c_x = x * dot_product * 2.0f;
        float c_y = y * dot_product * 2.0f;
        float c_z = z * dot_product * 2.0f;
        // Compute final result = a - b + c
        Vec3f projected_gravity;
        projected_gravity[0] = (a_x - b_x + c_x);
        projected_gravity[1] = (a_y - b_y + c_y);
        projected_gravity[2] = a_z - b_z + c_z;

        float new_joint_pos[12];
        float new_joint_vel[12];

        // 新顺序的索引映射
        int new_order[12] = {0,3,6,9,1,4,7,10,2,5,8,11};

        for (int i = 0; i < 12; i++) {
            new_joint_pos[i] = ro.joint_pos[new_order[i]];
            new_joint_vel[i] = ro.joint_vel[new_order[i]];
        }

        float dof_pos_default_obs[12] = {0, 0, 0, 0, -0.5, -0.5, -0.5, -0.5, 1.1, 1.1, 1.1, 1.1};


        std::vector<float> data;
        data.reserve(45);
        // 添加 base_omega (3)
        for (size_t i = 0; i < 3; ++i) {
            data.push_back(ro.base_omega[i]);
        }
        
        // 添加 base_rpy (3)
        for (size_t i = 0; i < 3; ++i) {
            data.push_back(projected_gravity[i]);
        }
        
        // 添加 cmd_vel (3)
        for (size_t i = 0; i < 3; ++i) {
            data.push_back(cmd_vel[i]);
        }
        
        // 添加 joint_pos (12)
        for (size_t i = 0; i < 12; ++i) {
            data.push_back(new_joint_pos[i] - dof_pos_default_obs[i]);
        }
        
        // 添加 joint_vel (12)
        for (size_t i = 0; i < 12; ++i) {
            data.push_back(new_joint_vel[i]);
        }
        
        // 添加 last_action (12)
        for (size_t i = 0; i < 12; ++i) {
            data.push_back(last_action[i]);
        }
            
        // 创建 (1, 1, 45) 的张量
        auto options = torch::TensorOptions().dtype(torch::kFloat32);
        torch::Tensor obs_tensor = torch::from_blob(data.data(), {1, 1, 45}, options).clone();
    
        if (run_cnt_ == 0) {
            for (int i = 0; i < 10; i++) {
                // 使用 LibTorch 的索引方法替换 Python 切片
                obs_history_tensor_.index({
                    torch::indexing::Slice(),  // 第 0 维（所有行）
                    i,                        // 第 1 维（第 `i` 列）
                    torch::indexing::Slice()  // 第 2 维（所有深度）
                }) = obs_tensor.squeeze(1);
            }
        };

        // 使用 slice 获取第1到第9个时间步 (形状变为 [1,9,45])
        obs_history_tensor_ = obs_history_tensor_.slice(1, 1, obs_history_tensor_.size(1));
        
        // 2. 在末尾添加新数据 (:,9,:)
        // 使用 cat 拼接张量 (形状恢复为 [1,10,45])
        obs_history_tensor_ = torch::cat({obs_history_tensor_, obs_tensor}, /*dim=*/1);

        // std::cout << "action_: " <<  obs_history_tensor_.sizes() << std::endl;
        // std::cout << "action_: " <<  obs_history_tensor_ << std::endl;

    
        std::vector<c10::IValue> inputs;
        inputs.push_back(c10::IValue(obs_history_tensor_));

        std::cout << "cmd_vel: " << cmd_vel << std::endl;
        // std::cout << "input: " << inputs << std::endl;

        test = backbone_.forward(inputs);


        auto tupleValue = test.toTuple();
        
        auto firstElement = tupleValue->elements()[0];

        // std::cout << "test: " << firstElement << std::endl;

        at::Tensor firstTensor = firstElement.toTensor();

        Eigen::Map<Eigen::MatrixXf> act(firstTensor.data_ptr<float>(), 1, 12);
        action_ = act.row(0);

        last_action = action_;

        // std::cout << "test: " << action_ << std::endl;

        action_ = 0.25 * action_ + dof_pos_default_;


        // std::cout << "dof_pos_default_: " << dof_pos_default_ << std::endl;
        
        // std::cout << "pos: " << action_ << std::endl;

        //顺序的索引映射
        int new_order_r[12] = {0,4,8,1,5,9,2,6,10,3,7,11};
     
        VecXf new_action_(12);
        for (int i = 0; i < 12; i++) {
            new_action_[i] = action_[new_order_r[i]];
        }

        // std::cout << "pos: " << new_action_ << std::endl;

        RobotAction ra;
        ra.goal_joint_pos = new_action_;
        ra.goal_joint_vel = VecXf::Zero(act_dim_);
        ra.tau_ff = VecXf::Zero(act_dim_);
        ra.kp = kp_;
        ra.kd = kd_;
        ++run_cnt_;
        return ra;
    }
};

#endif

