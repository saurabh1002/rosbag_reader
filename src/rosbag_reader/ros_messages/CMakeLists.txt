add_library(ros_msgs OBJECT std_msgs.cpp sensor_msgs.cpp)
target_compile_features(ros_msgs PRIVATE cxx_std_17)
target_include_directories(ros_msgs PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                           $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
