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
#pragma once

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "std_msgs.hpp"

namespace sensor_msgs {
struct PointField {
    std::string name;
    uint32_t offset{};
    uint8_t datatype{};
    uint32_t count{};

    static PointField parsePointField(std::ifstream& rosbag);
};

class FieldData {
public:
    FieldData(const sensor_msgs::PointField& field);
    virtual ~FieldData() = default;

    virtual double read(std::ifstream& rosbag) = 0;
    virtual uint32_t sizeofData() const = 0;

public:
    sensor_msgs::PointField field_;
};

template <class T>
class FieldData_TypeT : public FieldData {
public:
    explicit FieldData_TypeT(const sensor_msgs::PointField& field)
        : FieldData(field) {}

    double read(std::ifstream& rosbag) override {
        T data;
        rosbag.read(reinterpret_cast<char*>(&data), sizeof(data));
        return static_cast<double>(data);
    }
    uint32_t sizeofData() const override { return sizeof(T); }
};

std::vector<std::shared_ptr<sensor_msgs::FieldData>> getFields(
        const std::vector<sensor_msgs::PointField>& fields);

struct PointCloud2 {
    std_msgs::Header header;
    uint32_t height;
    uint32_t width;
    int32_t num_fields;
    std::vector<PointField> fields;

    bool is_bigendian;
    uint32_t point_step;
    uint32_t row_step;
    std::vector<std::vector<double>> data;

    bool is_dense;

    PointCloud2(std::ifstream& rosbag);
    void saveAsPLY(const std::string& output_path);
};

struct LaserScan {
    std_msgs::Header header;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;

    LaserScan(std::ifstream& rosbag);
    void saveAsPLY(const std::string& output_path);
};
}  // namespace sensor_msgs
