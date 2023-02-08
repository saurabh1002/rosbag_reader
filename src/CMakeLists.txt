add_subdirectory(ros_messages)

add_library(rosbag_reader STATIC rosbag.cpp)
target_include_directories(rosbag_reader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros_messages>
)
target_link_libraries(rosbag_reader PUBLIC ros_msgs)

add_executable(parsePointCloud2Msgs parsePointCloud2Msgs.cpp)
target_include_directories(parsePointCloud2Msgs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                            PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/utils>
                            PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/ros_messages>
                            )
target_link_libraries(parsePointCloud2Msgs PUBLIC rosbag_reader ros_msgs)
