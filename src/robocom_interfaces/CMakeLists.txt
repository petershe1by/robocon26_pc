cmake_minimum_required(VERSION 3.8)
project(robocom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/MathResult.msg"
  "msg/RobotState.msg"
  "msg/BlockInfo.msg"
  "msg/MotionCmd.msg"
  "msg/MissionStatus.msg"
  "msg/ArmCommand.msg"
)

set(srv_files
  "srv/SetCoordinate.srv"
  "srv/StartMission.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()
