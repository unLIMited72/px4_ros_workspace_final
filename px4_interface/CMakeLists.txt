cmake_minimum_required(VERSION 3.8)
project(px4_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PX4Status.msg"
  "msg/InstanceEvent.msg"
  "msg/InstanceList.msg"
  "msg/UIStatus.msg"
  "msg/MissionWaypoint.msg"
  "msg/MissionPlan.msg"
  "msg/MissionCommand.msg"
  "msg/MissionPlanForDrone.msg"
  "msg/MissionStatus.msg"
  "msg/GyroControl.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
