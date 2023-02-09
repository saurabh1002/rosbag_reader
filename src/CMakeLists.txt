add_subdirectory(ros_messages)
add_subdirectory(utils)

add_library(rosbag_reader STATIC)
target_sources(rosbag_reader PRIVATE rosbag.cpp)
set_target_properties(rosbag_reader PROPERTIES PUBLIC_HEADER "rosbag.h")
target_include_directories(rosbag_reader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                            PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/ros_messages>)
target_link_libraries(rosbag_reader PUBLIC ros_msgs)

install(TARGETS rosbag_reader
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
        PUBLIC_HEADER
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})