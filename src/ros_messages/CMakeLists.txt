add_library(ros_msgs STATIC)
target_sources(ros_msgs PRIVATE std_msgs.cpp sensor_msgs.cpp)
target_include_directories(ros_msgs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)