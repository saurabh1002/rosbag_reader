#include <pybind11/pybind11.h>

#include <string>

#include "rosbag_reader/ros_messages/ros_messages.h"
#include "rosbag_reader/rosbag.h"

namespace py = pybind11;
using namespace py::literals;
PYBIND11_MODULE(rosbag_reader_pybind, m) {
    py::class_<Rosbag> rosbag(m, "Rosbag", "HELP HERE");
    rosbag.def(py::init<std::string>(), "rosbag_path"_a)
            .def("print_info", &Rosbag::printInfo, "rosbag info equivalent");
}
