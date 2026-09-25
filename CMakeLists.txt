cmake_minimum_required(VERSION 3.8)
project(lidar_cluster_tracker LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find standard ROS2 ament build tools and packages
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(tracker_node
  src/tracker_node.cpp
  src/cluster_engine.c
)

target_include_directories(tracker_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(tracker_node
  rcl
  rclc
  sensor_msgs
  visualization_msgs
)

target_link_libraries(tracker_node PRIVATE m)

install(TARGETS tracker_node DESTINATION lib/${PROJECT_NAME})
ament_package()