#pragma once

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
} __attribute__((aligned(64))) __attribute__((packed));

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
        return double{data};
    }
    uint32_t sizeofData() const override { return sizeof(T); }
};

PointField parsePointField(std::ifstream& rosbag);

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
};

PointCloud2 parsePointCloud2(std::ifstream& rosbag);
}  // namespace sensor_msgs
