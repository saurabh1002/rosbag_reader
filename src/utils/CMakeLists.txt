add_library(utility STATIC)
target_sources(utility PRIVATE utils.cpp)
set_target_properties(utility PROPERTIES PUBLIC_HEADER "utils.h")
target_include_directories(utility PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} 
                        PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/ros_messages>)
target_link_libraries(utility PUBLIC ros_msgs)

install(TARGETS utility
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
        PUBLIC_HEADER
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})