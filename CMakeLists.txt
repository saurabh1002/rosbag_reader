# Specify minimum cmake version required
cmake_minimum_required(VERSION 3.12.0)
project(rosbag_reader VERSION 1.0
                      DESCRIPTION "Read ROSv1 bag files using plain C++" 
                      LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(BUILD_TESTS ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Specify build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/src)

if(BUILD_TESTS)
  add_subdirectory(${CMAKE_SOURCE_DIR}/test)
endif()
