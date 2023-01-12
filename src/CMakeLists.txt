add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ros_messages)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_library(rosbag STATIC rosbag.cpp)
target_include_directories(rosbag PUBLIC ${CMAKE_SOURCE_DIR}/include/utils)
target_link_libraries(rosbag PUBLIC sensor_msgs parse_util)

add_executable(parsePointCloud2Msgs parsePointCloud2Msgs.cpp)
target_link_libraries(parsePointCloud2Msgs PUBLIC parse_util rosbag)
target_include_directories(parsePointCloud2Msgs PUBLIC ${CMAKE_SOURCE_DIR}/include/)
