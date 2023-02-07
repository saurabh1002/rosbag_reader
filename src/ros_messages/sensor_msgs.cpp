#include "sensor_msgs.hpp"

#include <fstream>
#include <memory>
#include <vector>

#include "std_msgs.hpp"
#include "utils.hpp"

using sensor_msgs::FieldData;
using sensor_msgs::FieldData_TypeT;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointField;

FieldData::FieldData(const PointField &field, unsigned long num_points)
    : field_(field) {
    data_vec_.reserve(num_points);
}

PointField sensor_msgs::parsePointField(std::ifstream &rosbag) {
    PointField point_field;

    int32_t len_name = 0;
    rosbag.read(reinterpret_cast<char *>(&len_name), 4);
    utils::parser::readString(rosbag, point_field.name, len_name);
    rosbag.read(reinterpret_cast<char *>(&point_field.offset), 4);
    rosbag.read(reinterpret_cast<char *>(&point_field.datatype), 1);
    rosbag.read(reinterpret_cast<char *>(&point_field.count), 4);

    return std::move(point_field);
}

std::vector<std::shared_ptr<FieldData>> sensor_msgs::createFieldDataVec(
        const std::vector<PointField> &fields, const unsigned long num_points) {
    std::vector<std::shared_ptr<FieldData>> fields_data;
    fields_data.reserve(fields.size());

    for (const auto &field : fields) {
        switch (field.datatype) {
            case 1:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<int8_t>(field, num_points)));
                break;

            case 2:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<uint8_t>(field, num_points)));
                break;

            case 3:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<int16_t>(field, num_points)));
                break;

            case 4:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<uint16_t>(field, num_points)));
                break;

            case 5:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<int32_t>(field, num_points)));
                break;

            case 6:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<uint32_t>(field, num_points)));
                break;

            case 7:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<float>(field, num_points)));
                break;

            case 8:
                fields_data.emplace_back(std::shared_ptr<FieldData>(
                        new FieldData_TypeT<double>(field, num_points)));
                break;

            default:
                break;
        }
    }
    return fields_data;
}

PointCloud2 sensor_msgs::parsePointCloud2(std::ifstream &rosbag) {
    PointCloud2 pcl2;
    pcl2.header = std_msgs::parseHeader(rosbag);

    rosbag.read(reinterpret_cast<char *>(&pcl2.height), 4);
    rosbag.read(reinterpret_cast<char *>(&pcl2.width), 4);
    rosbag.read(reinterpret_cast<char *>(&pcl2.num_fields), 4);

    auto num_points = static_cast<unsigned long>(pcl2.height) *
                      static_cast<unsigned long>(pcl2.width);

    pcl2.fields.reserve(pcl2.num_fields);
    for (int i = 0; i < pcl2.num_fields; i++) {
        pcl2.fields.emplace_back(sensor_msgs::parsePointField(rosbag));
    }

    rosbag.read(reinterpret_cast<char *>(&pcl2.is_bigendian), sizeof(bool));
    rosbag.read(reinterpret_cast<char *>(&pcl2.point_step), 4);
    rosbag.read(reinterpret_cast<char *>(&pcl2.row_step), 4);

    int len_data_field = 0;
    rosbag.read(reinterpret_cast<char *>(&len_data_field), 4);

    // Read per point Data from the PointCloud2 Message
    auto fields_data = createFieldDataVec(pcl2.fields, num_points);
    uint32_t offset_ptr = 0;
    for (int i = 0; i < num_points; i++) {
        for (auto &field_data : fields_data) {
            rosbag.ignore(field_data->field_.offset - offset_ptr);
            field_data->read(rosbag);
            offset_ptr += (field_data->sizeofData() +
                           field_data->field_.offset - offset_ptr);
        }
        rosbag.ignore(pcl2.point_step - offset_ptr);
        offset_ptr = 0;
    }

    rosbag.read(reinterpret_cast<char *>(&pcl2.is_dense), sizeof(bool));

    pcl2.data.reserve(pcl2.num_fields);
    for (auto &field_data : fields_data) {
        pcl2.data.emplace_back(field_data->data_vec_);
    }

    pcl2.data = math::transpose(pcl2.data);
    return std::move(pcl2);
}
