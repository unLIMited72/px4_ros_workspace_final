cmake_minimum_required(VERSION 3.8)
project(px4_sim_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)


add_executable(px4_status_node src/px4_status_node.cpp)
ament_target_dependencies(px4_status_node rclcpp px4_msgs px4_interface tf2)

add_executable(mission_planner_node src/mission_planner_node.cpp)
ament_target_dependencies(mission_planner_node rclcpp px4_interface)

add_executable(mission_control_node src/mission_control_node.cpp)
ament_target_dependencies(mission_control_node rclcpp px4_msgs px4_interface)

add_executable(gcs_gyro_gateway_node src/gcs_gyro_gateway_node.cpp)
ament_target_dependencies(gcs_gyro_gateway_node rclcpp std_msgs px4_interface)
target_link_libraries(gcs_gyro_gateway_node nlohmann_json::nlohmann_json)

add_executable(gyro_control_node src/gyro_control_node.cpp)
ament_target_dependencies(gyro_control_node rclcpp px4_msgs px4_interface)

add_executable(px4_interface_bag_recorder src/px4_interface_bag_recorder.cpp)
ament_target_dependencies(px4_interface_bag_recorder rclcpp rosbag2_cpp rosbag2_storage px4_interface)

install(TARGETS
  px4_status_node
  mission_planner_node
  mission_control_node
  gcs_gyro_gateway_node
  gyro_control_node
  px4_interface_bag_recorder
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
