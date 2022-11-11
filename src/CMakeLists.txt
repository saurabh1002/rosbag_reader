add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC utils glog::glog indicators::indicators)
target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/includes/)