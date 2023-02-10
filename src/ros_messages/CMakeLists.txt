add_library(ros_msgs STATIC)
target_sources(ros_msgs PRIVATE std_msgs.cpp sensor_msgs.cpp)
set_target_properties(ros_msgs PROPERTIES PUBLIC_HEADER "ros_messages.h")
target_include_directories(ros_msgs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} 
                            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)

install(TARGETS ros_msgs
        EXPORT ${PROJECT_NAME}Targets
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
                