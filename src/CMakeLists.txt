add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ros_messages)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_executable(parsePointCloud2Msgs parsePointCloud2Msgs.cpp)
target_link_libraries(parsePointCloud2Msgs PUBLIC bag_utils glog::glog indicators::indicators yaml-cpp)
target_include_directories(parsePointCloud2Msgs PUBLIC ${CMAKE_SOURCE_DIR}/includes/)