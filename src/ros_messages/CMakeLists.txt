add_library(std_msgs STATIC ${CMAKE_CURRENT_SOURCE_DIR}/std_msgs.cpp)
target_include_directories(std_msgs PUBLIC ${CMAKE_SOURCE_DIR}/include/ros_messages
                                           ${CMAKE_SOURCE_DIR}/include/utils)

add_library(sensor_msgs STATIC ${CMAKE_CURRENT_SOURCE_DIR}/sensor_msgs.cpp)
target_include_directories(sensor_msgs PUBLIC ${CMAKE_SOURCE_DIR}/include/ros_messages
                                              ${CMAKE_SOURCE_DIR}/include/utils)
target_link_libraries(sensor_msgs PUBLIC std_msgs io_utils)
