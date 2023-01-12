add_library(io_utils STATIC ${CMAKE_CURRENT_SOURCE_DIR}/io.cpp)
target_include_directories(io_utils PUBLIC ${CMAKE_SOURCE_DIR}/include/utils)

add_library(parse_util STATIC ${CMAKE_CURRENT_SOURCE_DIR}/parse.cpp)
target_include_directories(parse_util PUBLIC ${CMAKE_SOURCE_DIR}/include/utils)
