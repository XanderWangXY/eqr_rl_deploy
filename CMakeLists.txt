cmake_minimum_required(VERSION 3.10)

project(rl_controller)

# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)
add_compile_options("-g")
add_definitions(-w) # warning ignore
add_compile_options(-fPIC)
set(BUILD_PLATFORM "x86" CACHE STRING "select build cpu type")
option(BUILD SIM OFF)
option(USE_RAISIM OFF)
set(USE_PYBULLET ON)
set(USE_RAISIM OFF)
option(SEND_REMOTE OFF)
if (BUILD_PLATFORM STREQUAL arm)
  message("this is arm platform")
  set(BUILD_SIM OFF)
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
else()
  message("this is x86 platform")
endif()
set(CMAKE_CXX_STANDARD 17)

if (SEND_REMOTE)
  set(BUILD_SIM OFF)
endif()

# # ✅ 新增：仿真选项（PyBullet/MuJoCo互斥）
# option(USE_PYBULLET "Enable PyBullet simulation" ON)
# option(USE_MUJOCO "Enable MuJoCo simulation" OFF)

# # 确保互斥：启用一个时自动关闭另一个
# if(USE_MUJOCO)
#     set(USE_PYBULLET OFF CACHE BOOL "Enable PyBullet simulation" FORCE)
# elseif(USE_PYBULLET)
#     set(USE_MUJOCO OFF CACHE BOOL "Enable MuJoCo simulation" FORCE)
# endif()

# # 确保至少选择一个仿真或硬件模式
# if(NOT BUILD_SIM AND NOT USE_PYBULLET AND NOT USE_MUJOCO)
#     message(FATAL_ERROR "Either BUILD_SIM (hardware) or USE_PYBULLET/MUJOCO (simulation) must be enabled")
# endif()

# # # ✅ 新增：MuJoCo依赖检查（需提前安装MuJoCo和GLFW）
# # if(USE_MUJOCO)
# #     find_package(mujoco REQUIRED)
# #     find_package(glfw3 REQUIRED)
# #     add_definitions(-DUSE_MUJOCO)
# # endif()

message("BUILD_PLATFORM: ${BUILD_PLATFORM}")
message("BUILD_SIM:      ${BUILD_SIM}")
message("SEND_REMOTE:    ${SEND_REMOTE}")

set(ROBOT_TYPE "Lite3" CACHE STRING "Select target robot type (Lite3, eqr1, Go2)")
set_property(CACHE ROBOT_TYPE PROPERTY STRINGS "Lite3" "eqr1" "Go2") # 可选类型
message("Selected Robot Type: ${ROBOT_TYPE}")

add_definitions(-DROBOT_TYPE_${ROBOT_TYPE})

get_filename_component(WORKSPACE_DIR ./ ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/third_party)

# ✅ 新增：根据机器人类型动态设置SDK路径和库文件
if(ROBOT_TYPE STREQUAL "Lite3")
    set(MOTION_SDK_PATH "${THIRD_PARTY}/MotionSDK/Lite3_MotionSDK")
    # set(HARDWARE_LIBRARY "${MOTION_SDK_PATH}/liblite3_hardware.so")  # Lite3的库文件
elseif(ROBOT_TYPE STREQUAL "eqr1")
    set(MOTION_SDK_PATH "${THIRD_PARTY}/MotionSDK/eqr1_MotionSDK")
    set(ROBOT_LIBS "${MOTION_SDK_PATH}/libeqr_hardware.so")    # EQR的库文件
elseif(ROBOT_TYPE STREQUAL "Go2")
    set(MOTION_SDK_PATH "${THIRD_PARTY}/MotionSDK/unitree_sdk2")
    list(APPEND CMAKE_PREFIX_PATH "${MOTION_SDK_PATH}")
    find_package(unitree_sdk2 CONFIG REQUIRED)
    set(ROBOT_LIBS unitree_sdk2)        
else()
    message(FATAL_ERROR "Unsupported robot type: ${ROBOT_TYPE}")
endif()

set(Torch_DIR "${THIRD_PARTY}/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
message("Torch_DIR:  ${Torch_DIR}")

include_directories(
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${MOTION_SDK_PATH}/include  # ✅ 动态替换为当前机器人的SDK头文件路径
  types
  utils
  interface/robot
  interface/user_command
  state_machine
  run_policy
  ${TORCH_INCLUDE_DIRS}
)

add_subdirectory(interface)

if (BUILD_SIM)
message("USE_PYBULLET:    ${USE_PYBULLET}")

  if(USE_RAISIM)
    add_definitions(-DUSE_RAISIM)
    set(raisim_DIR "your path to raisim")
    find_package(raisim CONFIG REQUIRED)
  elseif(USE_PYBULLET)
    add_definitions(-DUSE_PYBULLET)
  # elseif(USE_MUJOCO)
  #   add_definitions(-DUSE_MUJOCO)
  endif()
endif()

if (SEND_REMOTE)
  set(PATH_LIB "../lib")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  set(CMAKE_INSTALL_RPATH ${PATH_LIB}/libtorch/${BUILD_PLATFORM}/lib)
endif()


file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")
file(GLOB_RECURSE RUN_POLICY_SRC "run_policy/*.cpp")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC} ${RUN_POLICY_SRC})
target_link_libraries(rl_deploy interface -lpthread -lm -lrt -ldl -lstdc++ torch ${ROBOT_LIBS})

if (BUILD_SIM)
  if(USE_RAISIM)
    message("build simulation")
    target_link_libraries(rl_deploy raisim::raisim)
  endif()
endif()

if (SEND_REMOTE)
  add_custom_target(push_to_robot ALL DEPENDS rl_deploy)
  add_custom_command(
    TARGET push_to_robot POST_BUILD
    COMMAND ./scripts/sftp_to_remote.sh
    WORKING_DIRECTORY ..
    COMMENT "run sftp_to_remote.sh"
  )
endif()
