add_library(ros_msgs OBJECT std_msgs.cpp sensor_msgs.cpp)
target_compile_features(ros_msgs PRIVATE cxx_std_17)
set_target_properties(ros_msgs PROPERTIES PUBLIC_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/ros_messages.h")
target_include_directories(ros_msgs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} 
                            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
                            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                            )