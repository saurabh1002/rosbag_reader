add_subdirectory(ros_messages)
add_subdirectory(utils)

add_library(rosbag_reader STATIC)
target_sources(rosbag_reader PRIVATE rosbag.cpp)
set_target_properties(rosbag_reader PROPERTIES PUBLIC_HEADER "rosbag.h")
target_include_directories(rosbag_reader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                            PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/ros_messages>)
target_link_libraries(rosbag_reader PUBLIC ros_msgs)

install(TARGETS rosbag_reader
        EXPORT ${PROJECT_NAME}Targets
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT "${PROJECT_NAME}Targets" FILE "${PROJECT_NAME}Targets.cmake" NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/${PROJECT_NAME})

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" VERSION "${version}"
        COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        INSTALL_DESTINATION lib/cmake/${PROJECT_NAME})


install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT "${PROJECT_NAME}Targets" FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Targets.cmake"
        NAMESPACE ${PROJECT_NAME}::)