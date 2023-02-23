find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(rosbag_reader_pybind MODULE rosbag_reader_pybind.cpp)
install(TARGETS rosbag_reader_pybind DESTINATION .)