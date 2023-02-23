add_subdirectory(ros_messages)
if(TRUE)
add_subdirectory(pybind)
endif()

add_library(rosbag_reader STATIC)
target_sources(rosbag_reader PRIVATE rosbag.cpp)
target_compile_features(rosbag_reader PRIVATE cxx_std_17)
set(PUBLIC_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/rosbag.h;${CMAKE_CURRENT_SOURCE_DIR}/ros_messages/ros_messages.h")
set_target_properties(rosbag_reader PROPERTIES PUBLIC_HEADER "${PUBLIC_HEADERS}")
target_include_directories(rosbag_reader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                           PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros_messages>)
target_link_libraries(rosbag_reader PUBLIC $<TARGET_OBJECTS:ros_msgs>)
include(cmake/install.cmake)
