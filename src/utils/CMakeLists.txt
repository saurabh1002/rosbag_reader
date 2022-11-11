add_library(header_msg STATIC ${CMAKE_CURRENT_SOURCE_DIR}/header.cpp)
add_library(pointfield_msg STATIC ${CMAKE_CURRENT_SOURCE_DIR}/pointField.cpp)
add_library(pcl2_msg STATIC ${CMAKE_CURRENT_SOURCE_DIR}/pointcloud2.cpp)
add_library(savepcd STATIC ${CMAKE_CURRENT_SOURCE_DIR}/savePCD.cpp)

target_include_directories(header_msg PUBLIC ${CMAKE_SOURCE_DIR}/includes)
target_include_directories(pointfield_msg PUBLIC ${CMAKE_SOURCE_DIR}/includes)
target_include_directories(savepcd PUBLIC ${CMAKE_SOURCE_DIR}/includes)

target_link_libraries(pcl2_msg PUBLIC pointfield_msg header_msg savepcd)

add_library(utils STATIC ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
target_link_libraries(utils PUBLIC pcl2_msg glog::glog)
target_include_directories(utils PUBLIC ${CMAKE_SOURCE_DIR}/includes/)
