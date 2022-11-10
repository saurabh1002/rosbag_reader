#include <algorithm>
#include <any>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "header.hpp"
#include "pointField.hpp"
#include "utils.hpp"

int readPointCloud2(std::ifstream &rosbag, int data_len) {
    header::read(rosbag, data_len);

    uint32_t height = 0;
    rosbag.read(reinterpret_cast<char *>(&height), sizeof(height));
    std::cout << "height: " << height << "\n";
    uint32_t width = 0;
    rosbag.read(reinterpret_cast<char *>(&width), sizeof(width));
    std::cout << "width: " << width << "\n";
    data_len -= static_cast<int>(sizeof(height) + sizeof(width));

    std::cout << "fields:\n";
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
    std::printf("is_bigendian: %d\n", static_cast<int>(is_bigendian));
    uint32_t point_step = 0;
    rosbag.read(reinterpret_cast<char *>(&point_step), sizeof(point_step));
    uint32_t row_step = 0;
    rosbag.read(reinterpret_cast<char *>(&row_step), sizeof(row_step));
    std::printf("point_step: %u\n", point_step);
    std::printf("row_step: %u\n", row_step);

    data_len -= (static_cast<int>(sizeof(row_step) + sizeof(point_step)) +
                 static_cast<int>(sizeof(is_bigendian)));

    int len_data_field = 0;
    rosbag.read(reinterpret_cast<char *>(&len_data_field),
                sizeof(len_data_field));
    data_len -= 4;
    std::cout << "len(data[]): " << len_data_field << "\n";

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

    std::vector<std::vector<double>> pointcloud2;
    std::for_each(fields_ptr.cbegin(), fields_ptr.cend(),
                  [&pointcloud2](auto field_ptr) {
                      pointcloud2.emplace_back(field_ptr->getData());
                  });

    std::ofstream out;
    out.open("pcl.txt");
    std::for_each(pointcloud2.cbegin(), pointcloud2.cend(),
                  [&out](auto field_vec) {
                      std::for_each(field_vec.cbegin(), field_vec.cend(),
                                    [&out](auto val) { out << val << ","; });
                      out << "\n";
                  });
    return 0;
}
