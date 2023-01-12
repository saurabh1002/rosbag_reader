add_subdirectory(ros_messages)
add_subdirectory(utils)

add_library(rosbag STATIC rosbag.cpp)
target_link_libraries(rosbag PUBLIC ros_msgs utils)

add_executable(parsePointCloud2Msgs parsePointCloud2Msgs.cpp)
target_link_libraries(parsePointCloud2Msgs PUBLIC rosbag ros_msgs utils)
