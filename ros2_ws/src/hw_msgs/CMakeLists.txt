cmake_minimum_required(VERSION 3.8)
project(hw_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(MSG_FILES
  msg/MotorState.msg
  msg/MotorStateArray.msg
  msg/MotorCommand.msg
  msg/MotorCommandArray.msg
  msg/BoardState.msg
  msg/PumpState.msg
  msg/PumpCommand.msg
  msg/CameraSync.msg
  msg/DetectedBlock.msg
  msg/DetectedBlockArray.msg
  msg/GraspTarget.msg
  msg/TaskStatus.msg
  msg/ArmTarget.msg
  msg/ArmStatus.msg
)

set(SRV_FILES
  srv/SetMotorMode.srv
  srv/SetMotorGains.srv
  srv/CalibrateMotor.srv
  srv/SetPump.srv
  srv/ConfigureCamera.srv
  srv/SaveParams.srv
  srv/StartTask.srv
  srv/AbortTask.srv
  srv/CalibrateMapping.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
