#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <string>
#include <vector>

#include "rosbag_reader/ros_messages/ros_messages.h"
#include "rosbag_reader/rosbag.h"
// #include "stl_vector.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MODULE(rosbag_reader_py, m) {
    py::bind_vector<std::vector<std::vector<double>>>(m, "DoubleVector2D");
    py::class_<Rosbag> rosbag(m, "Rosbag",
                              "Read PointCloud2 messages from a Rosbag File");
    rosbag.def(py::init<std::string>(), "rosbag_path"_a)
            .def("print_info", &Rosbag::printInfo, "Same as rosbag info")
            .def("read_data", &Rosbag::readData,
                 "Read Raw Data from the bag file and store it for further "
                 "processing as particular message types")
            .def("num_msgs_on_topic",
                 py::overload_cast<const std::string&>(
                         &Rosbag::getNumMsgsonTopic),
                 "topic_name"_a,
                 "Print the number of messages on the given topic")
            .def("extract_pcl2",
                 py::overload_cast<const std::string&, int>(
                         &Rosbag::extractPointCloud2),
                 "topic_name"_a, "idx"_a,
                 "Extract specific indexed Pointcloud on a topic");
}
