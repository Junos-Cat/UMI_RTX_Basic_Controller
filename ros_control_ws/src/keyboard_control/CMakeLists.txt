cmake_minimum_required(VERSION 3.8)
project(keyboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(nodePositionControl1 src/command_publisher.cpp)
ament_target_dependencies(nodePositionControl1 rclcpp std_msgs geometry_msgs)

add_executable(nodePositionControl2 src/keyboard_control.cpp)
ament_target_dependencies(nodePositionControl2 rclcpp sensor_msgs)

add_executable(nodeJointControl src/keyboard_joint_control.cpp)
ament_target_dependencies(nodeJointControl rclcpp sensor_msgs)

install(TARGETS
  nodePositionControl1
  nodePositionControl2
  nodeJointControl
  DESTINATION lib/${PROJECT_NAME})

ament_package()
