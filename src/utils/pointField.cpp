#include "pointField.hpp"

#include <glog/logging.h>

#include <memory>
#include <string>
#include <vector>

#include "utils.hpp"

void PointField::insertPointField(
        const std::string &name,
        uint32_t offset,
        const uint8_t datatype,
        const unsigned long size,
        std::vector<std::shared_ptr<PointField>> &fields_ptr) {
    switch (datatype) {
        case 1:
            fields_ptr.emplace_back(std::shared_ptr<PointField>(
                    new TypedPointField<int8_t>(name, offset, size)));
            break;

        case 2:
            fields_ptr.emplace_back(std::shared_ptr<PointField>(
                    new TypedPointField<uint8_t>(name, offset, size)));
            break;

        case 3:
            fields_ptr.emplace_back(std::shared_ptr<PointField>(
                    new TypedPointField<int16_t>(name, offset, size)));
            break;

        case 4:
            fields_ptr.emplace_back(std::shared_ptr<PointField>(
                    new TypedPointField<uint16_t>(name, offset, size)));
            break;

        case 5:
            fields_ptr.emplace_back(std::shared_ptr<PointField>(
                    new TypedPointField<int32_t>(name, offset, size)));
            break;

        case 6:
            fields_ptr.emplace_back(std::shared_ptr<PointField>(
                    new TypedPointField<uint32_t>(name, offset, size)));
            break;

        case 7:
            fields_ptr.emplace_back(std::shared_ptr<PointField>(
                    new TypedPointField<float>(name, offset, size)));
            break;

        case 8:
            fields_ptr.emplace_back(std::shared_ptr<PointField>(
                    new TypedPointField<double>(name, offset, size)));
            break;
    }
}

std::vector<std::shared_ptr<PointField::PointField>> PointField::read(
        std::ifstream &rosbag,
        int &data_len,
        unsigned long size,
        int32_t num_point_fields) {
    int32_t len_name = 0;
    uint32_t offset = 0;
    uint8_t datatype = 0;
    uint32_t count = 0;

    std::vector<std::shared_ptr<PointField>> fields_ptr;
    fields_ptr.reserve(num_point_fields);

    for (int i = 0; i < num_point_fields; i++) {
        std::string name;
        rosbag.read(reinterpret_cast<char *>(&len_name), sizeof(len_name));
        readString(rosbag, name, len_name);
        rosbag.read(reinterpret_cast<char *>(&offset), sizeof(offset));
        rosbag.read(reinterpret_cast<char *>(&datatype), sizeof(datatype));
        rosbag.read(reinterpret_cast<char *>(&count), sizeof(count));
        LOG(INFO) << "\t-\n\t\tname: " << name.c_str();
        LOG(INFO) << "\t-\n\t\toffset: " << offset;
        LOG(INFO) << "\t-\n\t\tdatatype: " << datatype;
        LOG(INFO) << "\t-\n\t\tcount: " << count;

        insertPointField(name, offset, datatype, size, fields_ptr);

        data_len -= (13 + static_cast<int>(len_name));
    }

    return fields_ptr;
}