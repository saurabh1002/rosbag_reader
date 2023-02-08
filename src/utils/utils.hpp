#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs.hpp"

namespace utils {
namespace parser {
inline void readString(std::ifstream &rosbag,
                       std::string &str,
                       const int n_bytes) {
    std::unique_ptr<char[]> const buffer(new char[n_bytes + 1]);
    buffer.get()[n_bytes] = '\0';
    rosbag.read(buffer.get(), n_bytes);
    str = buffer.get();
}
}  // namespace parser

namespace io {
inline void savePointCloudAsPLY(const sensor_msgs::PointCloud2 &pcl,
                                const std::string &output_path,
                                int idx) {
    const std::filesystem::path out_path(output_path);
    if (!std::filesystem::is_directory(out_path)) {
        std::filesystem::create_directories(out_path);
    }

    std::ofstream ply_out;
    const std::string filename = std::to_string(idx) + ".ply";
    ply_out.open(out_path / filename,
                 std::ios_base::out | std::ios_base::binary);

    ply_out << "ply\n"
            << "format binary_little_endian 1.0\n";
    ply_out << "element vertex " << pcl.data.size() << "\n";
    for (const auto &field : pcl.fields) {
        ply_out << "property double " << field.name << "\n";
    }
    ply_out << "end_header\n";

    for (const auto &point : pcl.data) {
        ply_out.write(reinterpret_cast<const char *>(point.data()),
                      sizeof(point[0]) * point.size());
    }
}
}  // namespace io
}  // namespace utils