add_library(io_utils STATIC ${CMAKE_CURRENT_SOURCE_DIR}/io_utils.cpp)
target_include_directories(io_utils PUBLIC ${CMAKE_SOURCE_DIR}/includes/utils)

add_library(bag_utils STATIC ${CMAKE_CURRENT_SOURCE_DIR}/bag_utils.cpp)
target_link_libraries(bag_utils PUBLIC std_msgs sensor_msgs glog::glog)
target_include_directories(bag_utils PUBLIC ${CMAKE_SOURCE_DIR}/includes/utils)
