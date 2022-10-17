add_library(utils STATIC ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp)
target_include_directories(utils PUBLIC ${CMAKE_SOURCE_DIR}/includes/utils.hpp)