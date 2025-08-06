#include "ehr_hardware.h"
#include <iostream>
#include <memory>
#include <unistd.h>
#include <iomanip>
#include <vector>
#include <variant>
#include <limits>
#include <chrono>
#include <cmath>
#include <thread>
#define JOINT_NUM 12
#define PI 3.14159265359f

void PrintState(std::shared_ptr<EhrHardware> &ehr_hardware, ehr_body_state *state)
{
  std::cout << std::setw(10) << "Joint" << std::setw(10) << "Position" << std::setw(10) << "Velocity" << std::setw(10) << "Torque" << std::endl;
  for (int i = 0; i < state->joint_num; i++)
  {
    std::cout << std::setw(10) << i << std::setw(10) << state->q_joints[i] << std::setw(10) << state->v_joints[i] << std::setw(10) << state->f_joints[i] << std::endl;
  }

  std::cout << "+-------------+-- IMU Data -+------+------+" << std::endl;
  std::cout << "|-------------+-- original -+------+------|" << std::endl;
  std::cout << "| scaledAccel |" ;
  for(int i=0;i<3;i++){
    if(state->a_base[i] >=0)
      std::cout << " ";
    std::cout << state->a_base[i];
    if(std::abs(state->a_base[i]) <=10)
      std::cout << " ";
    std::cout << "|";  
  }
  std::cout << "      |";
  std::cout << std::endl;

  std::cout << "| scaledGyro  |" ;
    for(int i=0;i<3;i++){
        if(state->v_base[3+i] >=0)
          std::cout << " ";
        std::cout << state->v_base[3+i] << " |";
    }
    std::cout << "      |";
    //cout << " " << this->imu_data.Hz[1] << "Hz";
    std::cout << std::endl;

    std::cout << "| Quaternion  |" ;
    for(int i=0;i<4;i++){
        if(state->q_base[3+i] >=0)
          std::cout << " ";
        std::cout << state->q_base[3+i];
        if(std::abs(state->q_base[3+i]) <=10)
          if(i<3)
            std::cout << " ";
        std::cout << "|";
    }
    //cout << " " << this->imu_data.Hz[2] << "Hz";
    std::cout << std::endl;

    std::cout << "| Magnetometer|" ;
    for(int i=0;i<3;i++){
        if(state->magnetometer[i] >=0)
        std::cout << " ";
            std::cout << state->magnetometer[i];
        if(std::abs(state->magnetometer[i]) <=10)
          std::cout << " ";
        std::cout << "|";
    }
    std::cout << "      |";
    //cout << " " << this->imu_data.Hz[6] << "Hz";
    std::cout << std::endl;
    std::cout << "+-------------+------+------+------+------+" << std::endl;

 

}

void TEST_READ(std::shared_ptr<EhrHardware> &ehr_hardware, ehr_body_state *state)
{
  std::cout << "TEST_READ" << std::endl;
  ehr_hardware->GetInterface()->Read(state);
  PrintState(ehr_hardware, state);
}
void TEST_WRITE(std::shared_ptr<EhrHardware> &ehr_hardware, ehr_body_state *state)
{
  std::cout << "TEST_WRITE" << std::endl;
}

uint16_t angle_test = 0;
void TEST_LOOP(std::shared_ptr<EhrHardware> &ehr_hardware, ehr_body_state *state)
{
  std::cout << "TEST_WRITE_LOOP" << std::endl;
  for (int i = 0; i < JOINT_NUM; i++)
  {
    state->q_joint_desired[i] = 0.0;
    state->v_joint_desired[i] = 0.0;
    state->ff_joint_desired[i] = 0.0;
    state->kp_joints[i] = 0.0;
    state->kd_joints[i] = 1.0;
  }
  ehr_hardware->GetInterface()->Write(*state);


  ehr_hardware->GetInterface()->Read(state);
  double offset[JOINT_NUM] = {0.0};
  for (int i = 0; i < JOINT_NUM; i++)
  {
    offset[i] = state->q_joints[i];
  }
  auto start = std::chrono::high_resolution_clock::now();
  auto last_print = std::chrono::high_resolution_clock::now();
  while (true)
  {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start); // s
    float time_s = duration.count() / 1000.0;

    time_s *= 4;
    time_s = fmod(time_s, 2 * M_PI);

    double aim_pos = 3.1416 * sin(time_s * 6.28 / 5.0);
    double aim_vel = 1.28 / 5.0 * 3.1416 * cos(time_s * 6.28 / 5.0);

    // for (int i = 0; i < JOINT_NUM; i++)
    // {
    //   state->q_joint_desired[i] = aim_pos + offset[i];
    //   state->v_joint_desired[i] = 0;
    //   state->ff_joint_desired[i] = 0.0;
    //   state->kp_joints[i] = 0.0;
    //   state->kd_joints[i] = 0.5;
    // }

    for (int i = 0; i < JOINT_NUM; i++)
    {
      state->q_joint_desired[i] = 1*sin(PI*angle_test/180.0f); 
      state->v_joint_desired[i] = 1*cos(PI*angle_test/180.0f);  
      state->ff_joint_desired[i] = 0.5;
      state->kp_joints[i] = 2.0;
      state->kd_joints[i] = 0.5;
    }

    angle_test+= 1;
    ehr_hardware->GetInterface()->Write(*state);
    //sleep(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ehr_hardware->GetInterface()->Read(state);
    //PrintState(ehr_hardware, state);

    auto print_duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print);
    if(print_duration.count() > 500)
    {
      PrintState(ehr_hardware, state);
      //motor_stop_time++;
      last_print = now;
     // ehr_hardware.print();    // 打印电机信息
      //ehr_hardware.print_at10(); // 打印遥控器信息
      //ehr_hardware.print_imu();
      //ehr_hardware.print_battery();
      // if(motor_stop_time % 40 == 0)
      // {
      //     eir_motor.set_battery_cmd(Battery_48v, Battery_Open);
      //     printf("======open Battery_48v\n");
      // }else if(motor_stop_time % 20 == 0)
      // {
      //     eir_motor.set_battery_cmd(Battery_48v, Battery_Close);
      //     printf("======close Battery_48v\n");
      // }
      
      // // eir_motor.print_hand(-1);
      // if(bat_data.battery_status == 1)
      // {
      //     printf("bat_ctl_time=%d\n", bat_ctl_time);
      // }
    }
  }
}

int main(int argc, char **argv)
{
  std::shared_ptr<EhrHardware> ehr_hardware = std::make_shared<EhrHardware>();
  ehr_body_state state;
  ehr_hardware->GetInterface()->Init("./hardware_interface_ehr02.yaml");
  state.joint_num = JOINT_NUM;
  state.q_joints = new double[JOINT_NUM];
  state.v_joints = new double[JOINT_NUM];
  state.f_joints = new double[JOINT_NUM];
  state.gear_ratios = new double[JOINT_NUM];

  state.kp_joints = new double[JOINT_NUM];
  state.kd_joints = new double[JOINT_NUM];
  state.ki_joints = new double[JOINT_NUM];

  state.q_joint_desired = new double[JOINT_NUM];
  state.v_joint_desired = new double[JOINT_NUM];
  state.ff_joint_desired = new double[JOINT_NUM];
  
  // precision keep 3
  std::cout << std::fixed << std::setprecision(3);
  TEST_LOOP(ehr_hardware, &state);
  delete[] state.q_joints;
  delete[] state.v_joints;
  delete[] state.f_joints;
  delete[] state.gear_ratios;
  delete[] state.kp_joints;
  delete[] state.kd_joints;
  delete[] state.ki_joints;
  delete[] state.q_joint_desired;
  delete[] state.v_joint_desired;
  delete[] state.ff_joint_desired;
  return 0;
}