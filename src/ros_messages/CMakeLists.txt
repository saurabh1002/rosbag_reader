add_library(ros_msgs STATIC std_msgs.cpp sensor_msgs.cpp)
target_include_directories(ros_msgs PRIVATE ${CMAKE_SOURCE_DIR}/src/ros_messages
                           PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/utils>)