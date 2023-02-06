#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "std_msgs.hpp"

namespace math {
template <typename T>
using Matrix = std::vector<std::vector<T>>;

template <typename T>
Matrix<T> transpose(const Matrix<T>& vec) {
    if (vec.empty()) {
        return vec;
    }

    Matrix<T> trans_vec(vec[0].size(), std::vector<T>(vec.size()));

    for (int i = 0; i < vec.size(); i++) {
        for (int j = 0; j < vec[i].size(); j++) {
            trans_vec[j][i] = vec[i][j];
        }
    }
    return trans_vec;
}
}  // namespace math

namespace sensor_msgs {

struct PointField {
    std::string name;
    uint32_t offset{};
    uint8_t datatype{};
    uint32_t count{};
} __attribute__((aligned(64))) __attribute__((packed));

class FieldData {
public:
    FieldData(const sensor_msgs::PointField& field, unsigned long num_points);
    virtual ~FieldData() = default;

    virtual void read(std::ifstream& rosbag) = 0;
    virtual uint32_t sizeofData() const = 0;

public:
    sensor_msgs::PointField field_;
    std::vector<double> data_vec_;
};

template <class T>
class FieldData_TypeT : public FieldData {
public:
    explicit FieldData_TypeT(const sensor_msgs::PointField& field,
                             unsigned long num_points)
        : FieldData(field, num_points) {}

    void read(std::ifstream& rosbag) override {
        T data;
        rosbag.read(reinterpret_cast<char*>(&data), sizeof(data));
        data_vec_.emplace_back(double{data});
    }

    uint32_t sizeofData() const override { return sizeof(T); }
};

PointField parsePointField(std::ifstream& rosbag, int& data_len);

std::vector<std::shared_ptr<sensor_msgs::FieldData>> createFieldDataVec(
        const std::vector<sensor_msgs::PointField>& fields,
        unsigned long num_points);

struct PointCloud2 {
    std_msgs::Header header;
    uint32_t height;
    uint32_t width;
    int32_t num_fields;
    std::vector<PointField> fields;

    bool is_bigendian;
    uint32_t point_step;
    uint32_t row_step;
    math::Matrix<double> data;

    bool is_dense;
};

PointCloud2 parsePointCloud2(std::ifstream& rosbag, int data_len);
std::vector<PointCloud2> parsePointCloud2All(std::ifstream& rosbag,
                                             int data_len);
}  // namespace sensor_msgs