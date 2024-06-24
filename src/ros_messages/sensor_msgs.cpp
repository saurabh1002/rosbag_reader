#include <algorithm>
#include <fstream>
#include <memory>
#include <vector>

#include "ros_messages.h"

using sensor_msgs::FieldData;
using sensor_msgs::FieldData_TypeT;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointField;

FieldData::FieldData(const PointField &field) : field_(field) {}

PointField sensor_msgs::parsePointField(std::ifstream &rosbag) {
    PointField point_field;

    int32_t len_name = 0;
    rosbag.read(reinterpret_cast<char *>(&len_name), 4);
    point_field.name = std_msgs::readString(rosbag, len_name);
    rosbag.read(reinterpret_cast<char *>(&point_field.offset), 4);
    rosbag.read(reinterpret_cast<char *>(&point_field.datatype), 1);
    rosbag.read(reinterpret_cast<char *>(&point_field.count), 4);

    return std::move(point_field);
}

std::vector<std::shared_ptr<FieldData>> sensor_msgs::getFields(
        const std::vector<PointField> &fields) {
    std::vector<std::shared_ptr<FieldData>> fields_data;
    fields_data.reserve(fields.size());

    for (const auto &field : fields) {
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
    auto fields_data = getFields(pcl2.fields);
    std::vector<std::vector<double>> points;
    points.reserve(num_points);
    uint32_t offset_ptr = 0;
    for (int i = 0; i < num_points; i++) {
        std::vector<double> point;
        point.reserve(pcl2.num_fields);
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
        rosbag.ignore(pcl2.point_step - offset_ptr);
        offset_ptr = 0;
        if (!has_nan) {
            points.emplace_back(point);
        }
    }
    rosbag.read(reinterpret_cast<char *>(&pcl2.is_dense), sizeof(bool));
    pcl2.data = std::move(points);
    return std::move(pcl2);
}

LaserScan sensor_msgs::parseLaserScan(std::ifstream &rosbag) {
    LaserScan scan;
    scan.header = std_msgs::parseHeader(rosbag);
    rosbag.read(reinterpret_cast<char *>(&scan.angle_min), 4);
    rosbag.read(reinterpret_cast<char *>(&scan.angle_max), 4);
    rosbag.read(reinterpret_cast<char *>(&scan.angle_increment), 4);
    rosbag.read(reinterpret_cast<char *>(&scan.time_increment), 4);
    rosbag.read(reinterpret_cast<char *>(&scan.scan_time), 4);
    rosbag.read(reinterpret_cast<char *>(&scan.range_min), 4);
    rosbag.read(reinterpret_cast<char *>(&scan.range_max), 4);

    int len_ranges_field = 0;
    rosbag.read(reinterpret_cast<char *>(&len_ranges_field), 4);

    scan.ranges = std::vector<float>(len_ranges_field, 0.0f);
    std::for_each(scan.ranges.begin(), scan.ranges.end(), [&](auto &range) {
        rosbag.read(reinterpret_cast<char *>(&range), 4);
    });

    int len_intensities_field = 0;
    rosbag.read(reinterpret_cast<char *>(&len_intensities_field), 4);

    if (len_intensities_field > 0) {
        scan.intensities = std::vector<float>(len_intensities_field, 0.0f);
        std::for_each(scan.intensities.begin(), scan.intensities.end(),
                      [&](auto &intensity) {
                          rosbag.read(reinterpret_cast<char *>(&intensity), 4);
                      });
    }
    return std::move(scan);
}
