# CMake arguments for configuring ExternalProjects.
set(ExternalProject_CMAKE_ARGS
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON)

if(SILENCE_WARNINGS)
  set(ExternalProject_CMAKE_CXX_FLAGS "-DCMAKE_CXX_FLAGS=-w")
endif()

if(USE_SYSTEM_EIGEN3)
  find_package(Eigen3 QUIET NO_MODULE)
endif()
if(NOT USE_SYSTEM_EIGEN3 OR NOT EIGEN3_FOUND)
  set(USE_SYSTEM_EIGEN3 OFF)
  include(${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake)
endif()

find_package(Open3D REQUIRED)

include(${CMAKE_CURRENT_LIST_DIR}/argparse/argparse.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/yaml-cpp/yaml-cpp.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/indicators/indicators.cmake)