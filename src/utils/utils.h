#pragma once

#include <string>

#include "ros_messages.h"

namespace utils::io {
void savePointCloudAsPLY(const sensor_msgs::PointCloud2 &pcl,
                         const std::string &output_path,
                         int idx);
}  // namespace utils::io