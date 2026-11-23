cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(avoid_wall)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# Find the Gazebo_Transport library
find_package(gz-transport QUIET REQUIRED OPTIONAL_COMPONENTS log)

# ROS 2, for talking to robot_gui.py (source /opt/ros/jazzy/setup.bash first)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# JSON parsing for /robot/command (apt package libjsoncpp-dev)
find_package(jsoncpp REQUIRED)

include_directories(${CMAKE_BINARY_DIR})

if (EXISTS "${CMAKE_SOURCE_DIR}/lidar_node.cc")
  add_executable(lidar_node lidar_node.cc min_max_xy.cc)
  target_link_libraries(lidar_node
    gz-transport::core
    rclcpp::rclcpp
    ${std_srvs_TARGETS}
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    JsonCpp::JsonCpp)
endif()
