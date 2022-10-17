# Specify minimum cmake version required
cmake_minimum_required(VERSION 3.12.0)
project(rosbag_reader DESCRIPTION "Read ROS1 bag files using only c++" LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Specify build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Unit Test, Code Coverage, clang-format, clan-tidy, cppcheck Configuration:
option(ENABLE_CLANG_FORMAT "Enable static analysis with clang-format" OFF)
option(ENABLE_CLANG_TIDY "Enable static analysis with clang-tidy" OFF)
option(USE_SYSTEM_EIGEN "Use Eigen3 from the system" OFF)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

if (ENABLE_CLANG_TIDY)
  include(StaticAnalyzers)
endif(ENABLE_CLANG_TIDY)

if (ENABLE_CLANG_FORMAT)
  include(ClangFormat)
endif(ENABLE_CLANG_FORMAT)

# include(3rdparty/find_dependencies.cmake)

# Add subdirectory with a sub-project
add_subdirectory(${CMAKE_SOURCE_DIR}/src)
