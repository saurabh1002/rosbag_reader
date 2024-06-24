#pragma once

#include "utils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ros_messages.h"

void utils::io::savePointCloud2AsPLY(const sensor_msgs::PointCloud2 &pcl,
                                     const std::string &output_path,
                                     int idx) {
    const std::filesystem::path out_path(output_path);
    if (!std::filesystem::is_directory(out_path)) {
        std::filesystem::create_directories(out_path);
    }

    std::ofstream ply_out;
    std::ostringstream filename;
    filename << std::setw(5) << std::setfill('0') << idx << ".ply";
    ply_out.open(out_path / filename.str(),
                 std::ios_base::out | std::ios_base::binary);

    ply_out << "ply\n"
            << "format binary_little_endian 1.0\n";
    ply_out << "element vertex " << pcl.data.size() << "\n";
    for (const auto &field : pcl.fields) {
        ply_out << "property double " << field.name << "\n";
    }
    ply_out << "end_header\n";

    std::for_each(pcl.data.cbegin(), pcl.data.cend(), [&](const auto &point) {
        ply_out.write(reinterpret_cast<const char *>(point.data()),
                      sizeof(point[0]) * point.size());
    });
}

void utils::io::savePointCloud2AsBinary(const sensor_msgs::PointCloud2 &pcl,
                                        const std::string &output_path,
                                        int idx) {
    const std::filesystem::path out_path(output_path);
    if (!std::filesystem::is_directory(out_path)) {
        std::filesystem::create_directories(out_path);
    }

    std::ofstream bin_out;
    std::ostringstream filename;
    filename << std::setw(5) << std::setfill('0') << idx << ".bin";
    bin_out.open(out_path / filename.str(),
                 std::ios_base::out | std::ios_base::binary);

    std::for_each(pcl.data.cbegin(), pcl.data.cend(), [&](const auto &point) {
        bin_out.write(reinterpret_cast<const char *>(point.data()),
                      sizeof(point[0]) * 3);
    });
}

void utils::io::saveLaserScanAsPLY(const sensor_msgs::LaserScan &scan,
                                   const std::string &output_path,
                                   int idx) {
    const std::filesystem::path out_path(output_path);
    if (!std::filesystem::is_directory(out_path)) {
        std::filesystem::create_directories(out_path);
    }

    std::ofstream ply_out;
    std::ostringstream filename;
    filename << std::setw(5) << std::setfill('0') << idx << ".ply";
    ply_out.open(out_path / filename.str(),
                 std::ios_base::out | std::ios_base::binary);

    std::vector<std::array<float, 3>> scan_data;
    float angle = scan.angle_min;
    float scan_time = scan.scan_time;
    std::for_each(
            scan.ranges.cbegin(), scan.ranges.cend(), [&](const auto &range) {
                if ((range > scan.range_min) && (range < scan.range_max)) {
                    float x = range * std::cos(angle);
                    float y = range * std::sin(angle);
                    scan_data.emplace_back(std::array<float, 3>{x, y, scan_time});
                }
                scan_time += scan.time_increment;
                angle += scan.angle_increment;
            });

    ply_out << "ply\n"
            << "format binary_little_endian 1.0\n";
    ply_out << "element vertex " << scan_data.size() << "\n";

    ply_out << "property float x" << "\n";
    ply_out << "property float y" << "\n";
    ply_out << "property float time" << "\n";

    ply_out << "end_header\n";

    std::for_each(scan_data.cbegin(), scan_data.cend(), [&](const auto &point) {
        ply_out.write(reinterpret_cast<const char *>(point.data()),
                      sizeof(point[0]) * point.size());
    });
}

void utils::io::saveLaserScanAsBinary(const sensor_msgs::LaserScan &scan,
                                      const std::string &output_path,
                                      int idx) {
    const std::filesystem::path out_path(output_path);
    if (!std::filesystem::is_directory(out_path)) {
        std::filesystem::create_directories(out_path);
    }

    std::ofstream bin_out;
    std::ostringstream filename;
    filename << std::setw(5) << std::setfill('0') << idx << ".ply";
    bin_out.open(out_path / filename.str(),
                 std::ios_base::out | std::ios_base::binary);

    float angle = scan.angle_min;
    float scan_time = scan.scan_time;
    std::for_each(
            scan.ranges.cbegin(), scan.ranges.cend(), [&](const auto &range) {
                if ((range > scan.range_min) && (range < scan.range_max)) {
                    double x = range * std::cos(angle);
                    double y = range * std::sin(angle);
                    bin_out.write(reinterpret_cast<const char *>(&x),
                                  sizeof(x));
                    bin_out.write(reinterpret_cast<const char *>(&y),
                                  sizeof(y));
                    bin_out.write(reinterpret_cast<const char *>(&scan_time),
                                  sizeof(scan_time));
                }
                angle += scan.angle_increment;
                scan_time += scan.time_increment;
            });
}