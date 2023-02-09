add_executable(parse_pcl parse_pcl.cpp)
target_include_directories(parse_pcl PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/utils>)
target_link_libraries(parse_pcl PUBLIC rosbag_reader utility)