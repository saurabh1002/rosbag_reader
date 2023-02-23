cmake_minimum_required(VERSION 3.16...3.24)
project(rosbag_reader_cpp VERSION 0.2.1 LANGUAGES CXX)

add_subdirectory(ros_messages)
add_library(rosbag_reader STATIC)
target_sources(rosbag_reader PRIVATE rosbag.cpp)
target_compile_features(rosbag_reader PRIVATE cxx_std_17)
set(PUBLIC_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/rosbag.h;${CMAKE_CURRENT_SOURCE_DIR}/ros_messages/ros_messages.h")
set_target_properties(rosbag_reader PROPERTIES PUBLIC_HEADER "${PUBLIC_HEADERS}")
set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR})
get_filename_component(INCLUDE_DIRS ${INCLUDE_DIRS} PATH)
target_include_directories(
  rosbag_reader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
  PUBLIC $<BUILD_INTERFACE:${INCLUDE_DIRS}> $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rosbag_reader PUBLIC $<TARGET_OBJECTS:ros_msgs>)
# include(cmake/install.cmake)
