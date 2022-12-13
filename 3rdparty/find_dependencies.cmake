# CMake arguments for configuring ExternalProjects.
set(ExternalProject_CMAKE_ARGS
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON)

if(SILENCE_WARNINGS)
  set(ExternalProject_CMAKE_CXX_FLAGS "-DCMAKE_CXX_FLAGS=-w")
endif()

include(${CMAKE_CURRENT_LIST_DIR}/glog/glog.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/indicators/indicators.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/yaml-cpp/yaml-cpp.cmake)
