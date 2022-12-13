add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ros_messages)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_library(rosbag STATIC rosbag.cpp)
target_include_directories(rosbag PUBLIC ${CMAKE_SOURCE_DIR}/includes/utils)
target_link_libraries(rosbag PUBLIC sensor_msgs glog::glog parse_util)

add_executable(parsePointCloud2Msgs parsePointCloud2Msgs.cpp)
target_link_libraries(parsePointCloud2Msgs PUBLIC parse_util rosbag glog::glog indicators::indicators yaml-cpp)
target_include_directories(parsePointCloud2Msgs PUBLIC ${CMAKE_SOURCE_DIR}/includes/)
