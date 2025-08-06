#include "control_parameters.h"

void ControlParameters::GenerateGo2Parameters(){
    body_len_x_ = 0.1934*2;
    body_len_y_ = 0.0465*2;
    hip_len_ = 0.0955;
    thigh_len_ = 0.213;
    shank_len_ = 0.213;

    rotation = -1.0;
    offset = -0.3;

    pre_height_ = 0.30;
    stand_height_ = 0.30;
    swing_leg_kp_ << 100., 100., 100.;
    swing_leg_kd_ << 5, 5, 5;

    fl_joint_lower_ << -1.0472, -1.5708, -2.7227;
    fl_joint_upper_ << 1.0472, 3.4907, -0.83776;
    joint_vel_limit_ << 30.1, 30.1, 15.7;
    torque_limit_ << 23.7, 23.7, 45.43;
}
