#include <glog/logging.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "header.hpp"
#include "pointField.hpp"
#include "savePCD.hpp"
#include "utils.hpp"

int readPointCloud2(std::ifstream &rosbag, int data_len) {
    static int32_t count = 0;
    header::read(rosbag, data_len);

    uint32_t height = 0;
    rosbag.read(reinterpret_cast<char *>(&height), sizeof(height));
    LOG(INFO) << "height: " << height;
    uint32_t width = 0;
    rosbag.read(reinterpret_cast<char *>(&width), sizeof(width));
    LOG(INFO) << "width: " << width;
    data_len -= static_cast<int>(sizeof(height) + sizeof(width));

    LOG(INFO) << "fields:";
    int32_t num_point_fields = 0;
    rosbag.read(reinterpret_cast<char *>(&num_point_fields),
                sizeof(num_point_fields));
    data_len -= static_cast<int>(sizeof(num_point_fields));

    auto num_points = static_cast<unsigned long>(height) *
                      static_cast<unsigned long>(width);
    auto fields_ptr =
            PointField::read(rosbag, data_len, num_points, num_point_fields);

    bool is_bigendian = false;
    rosbag.read(reinterpret_cast<char *>(&is_bigendian), sizeof(is_bigendian));
    LOG(INFO) << "is_bigendian: " << static_cast<int>(is_bigendian);
    uint32_t point_step = 0;
    rosbag.read(reinterpret_cast<char *>(&point_step), sizeof(point_step));
    uint32_t row_step = 0;
    rosbag.read(reinterpret_cast<char *>(&row_step), sizeof(row_step));
    LOG(INFO) << "point_step: " << point_step;
    LOG(INFO) << "row_step: " << row_step;

    data_len -= (static_cast<int>(sizeof(row_step) + sizeof(point_step)) +
                 static_cast<int>(sizeof(is_bigendian)));

    int len_data_field = 0;
    rosbag.read(reinterpret_cast<char *>(&len_data_field),
                sizeof(len_data_field));
    data_len -= 4;
    LOG(INFO) << "length of data[]: " << len_data_field;

    uint32_t offset_ptr = 0;
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            for (int field_num = 0; field_num < num_point_fields; field_num++) {
                rosbag.ignore(fields_ptr[field_num]->getOffset() - offset_ptr);
                fields_ptr[field_num]->readDataFromStream(rosbag);
                offset_ptr += (fields_ptr[field_num]->sizeofData() +
                               fields_ptr[field_num]->getOffset() - offset_ptr);
            }
            rosbag.ignore(point_step - offset_ptr);
            offset_ptr = 0;
        }
    }

    bool is_dense = false;
    rosbag.read(reinterpret_cast<char *>(&is_dense), sizeof(is_dense));
    LOG(INFO) << "is_dense: " << static_cast<int>(is_dense);

    std::vector<std::vector<double>> pointcloud2;
    std::for_each(fields_ptr.cbegin(), fields_ptr.cend(),
                  [&pointcloud2](auto field_ptr) {
                      pointcloud2.emplace_back(field_ptr->getData());
                  });

    std::vector<std::string> field_names;
    field_names.reserve(num_point_fields);
    std::for_each(fields_ptr.cbegin(), fields_ptr.cend(),
                  [&field_names](auto field_ptr) {
                      field_names.emplace_back(field_ptr->getName());
                  });

    savePointCloud(pointcloud2, field_names, "../PLY", count);
    count++;
    return 0;
}
