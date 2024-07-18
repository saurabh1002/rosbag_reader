// MIT License

// Copyright (c) 2024 Saurabh Gupta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "messages.h"

namespace sensor_msgs {
FieldData::FieldData(const PointField &field) : field_(field) {}

PointField PointField::parsePointField(std::ifstream &rosbag) {
    PointField point_field;

    int32_t len_name = 0;
    rosbag.read(reinterpret_cast<char *>(&len_name), 4);
    point_field.name = std_msgs::readString(rosbag, len_name);
    rosbag.read(reinterpret_cast<char *>(&point_field.offset), 4);
    rosbag.read(reinterpret_cast<char *>(&point_field.datatype), 1);
    rosbag.read(reinterpret_cast<char *>(&point_field.count), 4);

    return std::move(point_field);
}

std::vector<std::shared_ptr<FieldData>> getFields(
        const std::vector<PointField> &fields) {
    std::vector<std::shared_ptr<FieldData>> fields_data;
    fields_data.reserve(fields.size());

    std::for_each(fields.cbegin(), fields.cend(), [&](const auto &field) {
        switch (field.datatype) {
            case 1:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<int8_t>(field)));
                break;

            case 2:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<uint8_t>(field)));
                break;

            case 3:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<int16_t>(field)));
                break;

            case 4:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<uint16_t>(field)));
                break;

            case 5:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<int32_t>(field)));
                break;

            case 6:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<uint32_t>(field)));
                break;

            case 7:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<float>(field)));
                break;

            case 8:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<double>(field)));
                break;

            default:
                break;
        }
    });
    return fields_data;
}

PointCloud2::PointCloud2(std::ifstream &rosbag) {
    header = std_msgs::Header::parseHeader(rosbag);

    rosbag.read(reinterpret_cast<char *>(&height), 4);
    rosbag.read(reinterpret_cast<char *>(&width), 4);
    rosbag.read(reinterpret_cast<char *>(&num_fields), 4);

    auto num_points = static_cast<unsigned long>(height) *
                      static_cast<unsigned long>(width);

    fields.reserve(num_fields);
    for (int i = 0; i < num_fields; i++) {
        fields.emplace_back(PointField::parsePointField(rosbag));
    }

    rosbag.read(reinterpret_cast<char *>(&is_bigendian), sizeof(bool));
    rosbag.read(reinterpret_cast<char *>(&point_step), 4);
    rosbag.read(reinterpret_cast<char *>(&row_step), 4);

    int len_data_field = 0;
    rosbag.read(reinterpret_cast<char *>(&len_data_field), 4);

    // Read per point Data from the PointCloud2 Message
    auto fields_data = getFields(fields);
    std::vector<std::vector<double>> points;
    points.reserve(num_points);
    uint32_t offset_ptr = 0;
    for (int i = 0; i < num_points; i++) {
        std::vector<double> point;
        point.reserve(num_fields);
        bool has_nan = false;
        std::for_each(
                fields_data.begin(), fields_data.end(), [&](auto &field_data) {
                    rosbag.ignore(field_data->field_.offset - offset_ptr);
                    double const data = field_data->read(rosbag);
                    if (data != data) {
                        has_nan = true;
                    }
                    point.emplace_back(data);
                    offset_ptr += (field_data->sizeofData() +
                                   field_data->field_.offset - offset_ptr);
                });
        rosbag.ignore(point_step - offset_ptr);
        offset_ptr = 0;
        if (!has_nan) {
            points.emplace_back(point);
        }
    }
    points.shrink_to_fit();
    rosbag.read(reinterpret_cast<char *>(&is_dense), sizeof(bool));
    data = std::move(points);
}

void PointCloud2::saveAsPLY(const std::string &output_path) {
    const std::filesystem::path out_path(output_path);
    if (!std::filesystem::is_directory(out_path)) {
        std::filesystem::create_directories(out_path);
    }

    std::ofstream ply_out;
    std::ostringstream filename;
    filename << std::to_string(header.stamp.secs) << "." << std::right
             << std::setfill('0') << std::setw(9)
             << std::to_string(header.stamp.nsecs) << ".ply";
    ply_out.open(out_path / filename.str(),
                 std::ios_base::out | std::ios_base::binary);

    ply_out << "ply" << std::endl
            << "format binary_little_endian 1.0" << std::endl;
    ply_out << "element vertex " << data.size() << std::endl;
    std::for_each(fields.cbegin(), fields.cend(), [&](const auto &field) {
        ply_out << "property double " << field.name << std::endl;
    });
    ply_out << "end_header" << std::endl;

    std::for_each(data.cbegin(), data.cend(), [&](const auto &point) {
        ply_out.write(reinterpret_cast<const char *>(point.data()),
                      sizeof(point[0]) * point.size());
    });
}

LaserScan::LaserScan(std::ifstream &rosbag) {
    header = std_msgs::Header::parseHeader(rosbag);
    rosbag.read(reinterpret_cast<char *>(&angle_min), 4);
    rosbag.read(reinterpret_cast<char *>(&angle_max), 4);
    rosbag.read(reinterpret_cast<char *>(&angle_increment), 4);
    rosbag.read(reinterpret_cast<char *>(&time_increment), 4);
    rosbag.read(reinterpret_cast<char *>(&scan_time), 4);
    rosbag.read(reinterpret_cast<char *>(&range_min), 4);
    rosbag.read(reinterpret_cast<char *>(&range_max), 4);

    int len_ranges_field = 0;
    rosbag.read(reinterpret_cast<char *>(&len_ranges_field), 4);

    ranges = std::vector<float>(len_ranges_field, 0.0f);
    std::for_each(ranges.begin(), ranges.end(), [&](auto &range) {
        rosbag.read(reinterpret_cast<char *>(&range), 4);
    });

    int len_intensities_field = 0;
    rosbag.read(reinterpret_cast<char *>(&len_intensities_field), 4);

    if (len_intensities_field > 0) {
        intensities = std::vector<float>(len_intensities_field, 0.0f);
        std::for_each(intensities.begin(), intensities.end(),
                      [&](auto &intensity) {
                          rosbag.read(reinterpret_cast<char *>(&intensity), 4);
                      });
    }
}

void LaserScan::saveAsPLY(const std::string &output_path) {
    const std::filesystem::path out_path(output_path);
    if (!std::filesystem::is_directory(out_path)) {
        std::filesystem::create_directories(out_path);
    }

    std::ofstream ply_out;
    std::ostringstream filename;
    filename << std::to_string(header.stamp.secs) << "." << std::right
             << std::setfill('0') << std::setw(9)
             << std::to_string(header.stamp.nsecs) << ".ply";
    ply_out.open(out_path / filename.str(),
                 std::ios_base::out | std::ios_base::binary);

    std::vector<std::array<float, 3>> scan_data;
    float angle = angle_min;
    float scan_time = scan_time;
    std::for_each(ranges.cbegin(), ranges.cend(), [&](const auto &range) {
        if ((range > range_min) && (range < range_max)) {
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);
            scan_data.emplace_back(std::array<float, 3>{x, y, scan_time});
        }
        scan_time += time_increment;
        angle += angle_increment;
    });

    ply_out << "ply" << std::endl
            << "format binary_little_endian 1.0" << std::endl;
    ply_out << "element vertex " << scan_data.size() << std::endl;

    ply_out << "property float x" << std::endl;
    ply_out << "property float y" << std::endl;
    ply_out << "property float time" << std::endl;

    ply_out << "end_header" << std::endl;

    std::for_each(scan_data.cbegin(), scan_data.cend(), [&](const auto &point) {
        ply_out.write(reinterpret_cast<const char *>(point.data()),
                      sizeof(point[0]) * point.size());
    });
}
}  // namespace sensor_msgs
