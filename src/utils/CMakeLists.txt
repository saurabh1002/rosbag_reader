add_library(utils STATIC io.cpp parse.cpp)
target_include_directories(utils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                           PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>)
