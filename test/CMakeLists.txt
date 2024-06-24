add_executable(parse_pcl2 parse_pcl2.cpp)
target_include_directories(parse_pcl2 PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/utils>)
target_link_libraries(parse_pcl2 PUBLIC rosbag_reader utility)

add_executable(parse_laserscan parse_laserscan.cpp)
target_include_directories(parse_laserscan PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/utils>)
target_link_libraries(parse_laserscan PUBLIC rosbag_reader utility)