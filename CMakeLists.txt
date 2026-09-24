cmake_minimum_required(VERSION 3.8)
project(lidar_cluster_tracker C)

# Find standard ROS2 ament build tools and packages
find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include the directory for header files (.h)
include_directories(include)

# Compile your custom source files into a single executable node
add_executable(tracker_node
  src/tracker_node.c
  src/cluster_engine.c
)

# Link your executable against ROS2 C-generator libraries and the Linux math library
target_link_libraries(tracker_node
  rclc::rclc
  sensor_msgs__rosidl_generator_c
  visualization_msgs__rosidl_generator_c
  m # Explicit link to the Linux math library for distance and tracking calculations
)

# Configure installation targets for colcon build workspace system
install(TARGETS tracker_node DESTINATION lib/${PROJECT_NAME})
ament_package()