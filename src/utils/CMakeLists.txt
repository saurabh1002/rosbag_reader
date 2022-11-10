add_library(header_msg STATIC ${CMAKE_CURRENT_SOURCE_DIR}/header.cpp)
add_library(pointfield_msg STATIC ${CMAKE_CURRENT_SOURCE_DIR}/pointField.cpp)
add_library(pcl2_msg STATIC ${CMAKE_CURRENT_SOURCE_DIR}/pointcloud2.cpp)

target_include_directories(header_msg PUBLIC ${CMAKE_SOURCE_DIR}/includes/header.hpp)
target_include_directories(pointfield_msg PUBLIC ${CMAKE_SOURCE_DIR}/includes/pointField.hpp)

target_link_libraries(pcl2_msg PUBLIC pointfield_msg header_msg)

add_library(utils STATIC ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
target_link_libraries(utils PUBLIC pcl2_msg)
target_include_directories(utils PUBLIC ${CMAKE_SOURCE_DIR}/includes/utils.hpp)

add_library(savepcd STATIC ${CMAKE_CURRENT_SOURCE_DIR}/savePCD.cpp)