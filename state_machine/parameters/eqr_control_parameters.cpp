#include "control_parameters.h"

void ControlParameters::GenerateEQR1Parameters(){
    body_len_x_ = 0.411;
    body_len_y_ = 0.05*2;
    hip_len_ = 0.0565;
    thigh_len_ = 0.20;
    shank_len_ = 0.2023;

    rotation = 1.0;
    offset = 0.0;

    pre_height_ = 0.25;
    stand_height_ = 0.35;
    swing_leg_kp_ << 100., 100., 100.;
    swing_leg_kd_ << 2.5, 2.5, 2.5;

    fl_joint_lower_ << -0.837, -3.49, 0.0;
    fl_joint_upper_ << 0.837, 2.042, 1.84;
    joint_vel_limit_ << 30, 30, 30;
    torque_limit_ << 40, 40, 65;
}
