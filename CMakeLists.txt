# Specify minimum cmake version required
cmake_minimum_required(VERSION 3.23)
project(rosbag_reader VERSION 1.0 DESCRIPTION "Read ROS1 bag files using plain C++" LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Specify build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_subdirectory(${CMAKE_SOURCE_DIR}/src)