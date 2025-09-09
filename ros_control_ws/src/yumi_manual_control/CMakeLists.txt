cmake_minimum_required(VERSION 3.5)
project(yumi_manual_control)

# C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

add_executable(node_manual_arm src/node_manual_arm.cpp)
ament_target_dependencies(node_manual_arm
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  node_manual_arm
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
