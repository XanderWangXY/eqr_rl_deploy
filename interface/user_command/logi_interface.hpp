#ifndef LOGI_INTERFACE_HPP_
#define LOGI_INTERFACE_HPP_

#include "user_command_interface.h"
#include "custom_types.h"
#include <cstdio>
#include <functional>
#include <termios.h>
#include <fcntl.h>    // For open()
#include <unistd.h>   // For close()
#include <stdio.h>    // For perror()

using namespace interface;
using namespace types;

class LogiInterface : public UserCommandInterface
{
private:
    UserCommand usr_cmd_;
    bool start_thread_flag_;
    std::thread logi_thread_;

    struct js_event {
        uint32_t time;     /* event timestamp in milliseconds */
        int16_t value;     /* value */
        uint8_t type;      /* event type */
        int8_t number;     /* axis/button number */
    };


    struct joystick_value
    {
        int fd;
        int8_t L1;
        int8_t L2; 
        int8_t R1;
        int8_t R2;
        int8_t L_up;
        int8_t L_down;
        int8_t L_left;
        int8_t L_right;
        int8_t X;
        int8_t A;
        int8_t B;
        int8_t Y;
        int8_t BACK;
        int8_t START;

        int16_t Lx_axis;
        int16_t Ly_axis;
        int16_t Rx_axis;
        int16_t Ry_axis;

        int16_t keys[14];
        int16_t axises[4];
    };

    struct joystick_value logi_f710;

    void js_map(struct joystick_value *js)
    {
        js->keys[0] = js->L1;js->keys[1] = js->L2;js->keys[2] = js->R1;js->keys[3] = js->R2;
        js->keys[4] = js->L_up;js->keys[5] = js->L_down;js->keys[6] = js->L_left;js->keys[7] = js->L_right;
        js->keys[8] = js->X;js->keys[9] = js->A;js->keys[10] = js->B;js->keys[11] = js->Y;
        js->keys[12] = js->BACK;js->keys[13] = js->START;
        js->axises[0] = js->Lx_axis;js->axises[1] = js->Ly_axis;
        js->axises[2] = js->Rx_axis;js->axises[3] = js->Ry_axis;
    }

    void js_unmap(struct joystick_value *js)
    {
        js->L1 = js->keys[0];js->L2 = js->keys[1];js->R1 = js->keys[2];js->R2 = js->keys[3];
        js->L_up = js->keys[4];js->L_down = js->keys[5];js->L_left = js->keys[6];js->L_right = js->keys[7];
        js->X = js->keys[8];js->A = js->keys[9];js->B = js->keys[10];js->Y = js->keys[11];
        js->BACK = js->keys[12];js->START = js->keys[13];
        js->Lx_axis = js->axises[0];js->Ly_axis = js->axises[1];
        js->Rx_axis = js->axises[2];js->Ry_axis = js->axises[3];
    }

    int joystick_read(struct joystick_value *js)
    {
        int fd = js->fd;
        fd_set fds;
        struct timeval timeout = {
            0,  // s
            0   // us
        };// 设置超时为0，立即返回，仅检测文件描述符是否就绪
        FD_ZERO(&fds);
        FD_SET(fd,&fds);
        int ret = select(fd+1,&fds,NULL,NULL,&timeout);
        if(ret == 0)
            return 0;
        else if(FD_ISSET(fd,&fds)){
            struct js_event event;
            read(fd,&event,sizeof(event));
            if(event.type == 1){
                switch(event.number){
                    case 0:js->X  = event.value;break;
                    case 1:js->A  = event.value;break;
                    case 2:js->B  = event.value;break;
                    case 3:js->Y  = event.value;break;
                    case 4:js->L1 = event.value;break;
                    case 5:js->R1 = event.value;break;
                    case 6:js->L2 = event.value;break;
                    case 7:js->R2 = event.value;break;
                    default:break;
                }
            }
            else if(event.type == 2){
                switch(event.number){
                    case 0:js->Lx_axis  = event.value;break;
                    case 1:js->Ly_axis  = -event.value;break;
                    case 2:js->Rx_axis  = event.value;break;
                    case 3:js->Ry_axis  = -event.value;break;
                    case 4:{
                        int left_padx = event.value;
                        if(left_padx < 0){
                            js->L_left = 1;
                            js->L_right = 0;
                        }
                        else if(left_padx > 0){
                            js->L_left = 0;
                            js->L_right = 1;
                        }
                        else{
                            js->L_left = 0;
                            js->L_right = 0;
                        }
                        break;
                    }
                    case 5:{
                        int left_pady = event.value;
                        if(left_pady < 0){
                            js->L_up = 1;
                            js->L_down = 0;
                        }
                        else if(left_pady > 0){
                            js->L_up = 0;
                            js->L_down = 1;
                        }
                        else{
                            js->L_up = 0;
                            js->L_down = 0;
                        }
                        break;
                    }
                    default:break;
                }
            }
            js_map(js);
            return 1;
        }
        return 0;
    }

public:

    LogiInterface(){
        std::memset(&usr_cmd_, 0, sizeof(usr_cmd_));
        std::cout << "Using Logitech joystick Command Interface" << std::endl;
        /* 打开JS设备 */
        int fd = open("/dev/input/js0",O_RDONLY);
        if(fd < 0){
            perror("can't open joystick device");
        }
        logi_f710.fd = fd;
    }

    ~LogiInterface(){}

    virtual void Start(){
        start_thread_flag_ = true;
        logi_thread_ = std::thread(std::bind(&LogiInterface::Run, this));
    }
    virtual void Stop(){
        start_thread_flag_ = false;
    }
    virtual UserCommand GetUserCommand(){return usr_cmd_;}

    virtual void SetMotionStateFeedback(const MotionStateFeedback& msfb){
        msfb_ = msfb;
    }


    void Run(){

        std::cout << "Start Logitech joystick Listening" << std::endl;
        while (start_thread_flag_) {
            joystick_read(&logi_f710);
            if(logi_f710.fd >= 0){
                if(logi_f710.L1 == 1){
                    usr_cmd_.target_mode = int(RobotMotionState::JointDamping);
                }
                // usr_cmd_.down_shift = false;
                // usr_cmd_.up_shift = false;
                switch(msfb_.current_state) {
                    case RobotMotionState::WaitingForStand:
                        if(logi_f710.Y == 1){
                            usr_cmd_.target_mode = int(RobotMotionState::StandingUp);
                        }
                    break;
                    case RobotMotionState::StandingUp:
                        if(logi_f710.R1 == 1){
                            usr_cmd_.target_mode = int(RobotMotionState::RLControlMode);
                        }
                    break;
                    case RobotMotionState::RLControlMode:
                        usr_cmd_.forward_vel_scale = logi_f710.Ly_axis/32768.0;
                        usr_cmd_.side_vel_scale = -logi_f710.Lx_axis/32768.0;
                        usr_cmd_.turnning_vel_scale = logi_f710.Ry_axis/32768.0;
                    break;
                    default:
                        break;
                }
            }
            // std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

};




#endif