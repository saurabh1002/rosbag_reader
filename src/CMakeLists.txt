add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ros_messages)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_executable(main read_pointcloud2.cpp)
target_link_libraries(main PUBLIC bag_utils glog::glog indicators::indicators yaml-cpp)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/includes/)