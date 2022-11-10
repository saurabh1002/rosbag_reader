#include "pointField.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "utils.hpp"

const std::map<uint8_t, uint32_t> dtype_to_bytes{
        {1, 1}, {2, 1}, {3, 2}, {4, 2}, {5, 4}, {6, 4}, {7, 4}, {8, 8}};

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
        std::printf("\t-\n\t\tname: %s", name.c_str());
        std::printf("\n\t\toffset: %d", offset);
        std::printf("\n\t\tdatatype: %d", datatype);
        std::printf("\n\t\tcount: %d\n", count);
        insertPointField(name, offset, datatype, size, fields_ptr);

        data_len -= (13 + static_cast<int>(len_name));
    }

    return fields_ptr;
}