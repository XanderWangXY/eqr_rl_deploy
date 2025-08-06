#include "state_machine.hpp"

using namespace types;

MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

int main(){
    // 根据CMake定义的预编译宏选择机器人类型
    RobotType robot_type;
#ifdef ROBOT_TYPE_Lite3
    robot_type = RobotType::Lite3;
#elif ROBOT_TYPE_eqr1
    robot_type = RobotType::eqr1;
#elif ROBOT_TYPE_Go2
    robot_type = RobotType::Go2;
#else
    // 默认或未定义时报错
    std::cerr << "Error: ROBOT_TYPE not defined. Please set ROBOT_TYPE in CMake." << std::endl;
    return 1;
#endif

    StateMachine state_machine(robot_type);
    state_machine.Run();
    return 0;
}