find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(rosbag_reader_py MODULE rosbag_reader_pybind.cpp)
target_link_libraries(rosbag_reader_py PRIVATE rosbag_reader)
install(TARGETS rosbag_reader_py DESTINATION .)
