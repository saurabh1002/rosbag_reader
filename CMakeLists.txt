cmake_minimum_required(VERSION 3.16...3.24)
project(rosbag_reader VERSION 1.0 DESCRIPTION "Read ROS1 bag files using plain C++" LANGUAGES CXX)

# Specify build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(src/rosbag_reader)
if(TRUE)
  add_subdirectory(src/pybind)
endif()

