#include "sensor_msgs.hpp"

#include <memory>
#include <string>
#include <vector>

#include "std_msgs.hpp"
#include "utils/io.hpp"  // TODO: Remove when we get rid of the PLY test application
#include "utils/parse.hpp"

void insertPointField(const std::string &name,
                      const uint32_t offset,
                      const uint8_t datatype,
                      const unsigned long size,
                      std::vector<std::shared_ptr<PointFieldMsg>> &fields_ptr) {
    switch (datatype) {
        case 1:
            fields_ptr.emplace_back(std::shared_ptr<PointFieldMsg>(
                    new TypedPointFieldMsg<int8_t>(name, offset, size)));
            break;

        case 2:
            fields_ptr.emplace_back(std::shared_ptr<PointFieldMsg>(
                    new TypedPointFieldMsg<uint8_t>(name, offset, size)));
            break;

        case 3:
            fields_ptr.emplace_back(std::shared_ptr<PointFieldMsg>(
                    new TypedPointFieldMsg<int16_t>(name, offset, size)));
            break;

        case 4:
            fields_ptr.emplace_back(std::shared_ptr<PointFieldMsg>(
                    new TypedPointFieldMsg<uint16_t>(name, offset, size)));
            break;

        case 5:
            fields_ptr.emplace_back(std::shared_ptr<PointFieldMsg>(
                    new TypedPointFieldMsg<int32_t>(name, offset, size)));
            break;

        case 6:
            fields_ptr.emplace_back(std::shared_ptr<PointFieldMsg>(
                    new TypedPointFieldMsg<uint32_t>(name, offset, size)));
            break;

        case 7:
            fields_ptr.emplace_back(std::shared_ptr<PointFieldMsg>(
                    new TypedPointFieldMsg<float>(name, offset, size)));
            break;

        case 8:
            fields_ptr.emplace_back(std::shared_ptr<PointFieldMsg>(
                    new TypedPointFieldMsg<double>(name, offset, size)));
            break;
    }
}

std::vector<std::shared_ptr<PointFieldMsg>> parsePointFieldMsg(
        std::ifstream &rosbag,
        int &data_len,
        const unsigned long size,
        const int32_t num_point_fields) {
    int32_t len_name = 0;
    uint32_t offset = 0;
    uint8_t datatype = 0;
    uint32_t count = 0;

    std::vector<std::shared_ptr<PointFieldMsg>> fields_ptr;
    fields_ptr.reserve(num_point_fields);

    for (int i = 0; i < num_point_fields; i++) {
        std::string name;
        rosbag.read(reinterpret_cast<char *>(&len_name), sizeof(len_name));
        readString(rosbag, name, len_name);
        rosbag.read(reinterpret_cast<char *>(&offset), sizeof(offset));
        rosbag.read(reinterpret_cast<char *>(&datatype), sizeof(datatype));
        rosbag.read(reinterpret_cast<char *>(&count), sizeof(count));

        insertPointField(name, offset, datatype, size, fields_ptr);

        data_len -= (13 + int{len_name});
    }

    return fields_ptr;
}

void parsePointCloud2Msg(std::ifstream &rosbag,
                         int data_len,
                         const std::string &topic,
                         const std::string &pcl_save_path) {
    static std::map<std::string, int32_t> count_per_topic;
    if (count_per_topic.find(topic) == count_per_topic.end()) {
        count_per_topic.insert({topic, 0});
    }

    parseHeaderMsg(rosbag, data_len);

    uint32_t height = 0;
    rosbag.read(reinterpret_cast<char *>(&height), sizeof(height));
    uint32_t width = 0;
    rosbag.read(reinterpret_cast<char *>(&width), sizeof(width));
    data_len -= int{sizeof(height) + sizeof(width)};

    int32_t num_point_fields = 0;
    rosbag.read(reinterpret_cast<char *>(&num_point_fields),
                sizeof(num_point_fields));
    data_len -= int{sizeof(num_point_fields)};

    auto num_points = static_cast<unsigned long>(height) *
                      static_cast<unsigned long>(width);
    auto fields_ptr =
            parsePointFieldMsg(rosbag, data_len, num_points, num_point_fields);

    bool is_bigendian = false;
    rosbag.read(reinterpret_cast<char *>(&is_bigendian), sizeof(is_bigendian));
    uint32_t point_step = 0;
    rosbag.read(reinterpret_cast<char *>(&point_step), sizeof(point_step));
    uint32_t row_step = 0;
    rosbag.read(reinterpret_cast<char *>(&row_step), sizeof(row_step));

    data_len -= (int{sizeof(row_step) + sizeof(point_step)} +
                 int{sizeof(is_bigendian)});

    int len_data_field = 0;
    rosbag.read(reinterpret_cast<char *>(&len_data_field),
                sizeof(len_data_field));
    data_len -= 4;

    uint32_t offset_ptr = 0;
    for (int i = 0; i < height * width; i++) {
        for (auto &field_ptr : fields_ptr) {
            rosbag.ignore(field_ptr->getOffset() - offset_ptr);
            field_ptr->readDataFromStream(rosbag);
            offset_ptr += (field_ptr->sizeofData() + field_ptr->getOffset() -
                           offset_ptr);
        }
        rosbag.ignore(point_step - offset_ptr);
        offset_ptr = 0;
    }

    bool is_dense = false;
    rosbag.read(reinterpret_cast<char *>(&is_dense), sizeof(is_dense));

    std::vector<std::string> field_names;
    std::vector<std::vector<double>> pointcloud2;
    field_names.reserve(num_point_fields);
    pointcloud2.reserve(num_point_fields);
    for (auto &field_ptr : fields_ptr) {
        pointcloud2.emplace_back(field_ptr->getData());
        field_names.emplace_back(field_ptr->getName());
    }

    savePointCloudAsPLY(pointcloud2, field_names, pcl_save_path + topic,
                        count_per_topic[topic]);
    count_per_topic[topic] += 1;
}
