#pragma once

#include "utils.h"

#include <filesystem>
#include <fstream>
#include <string>

#include "ros_messages.h"

void utils::io::savePointCloudAsPLY(const sensor_msgs::PointCloud2 &pcl,
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
