#pragma once

#include <string>

#include "ros_messages.h"

namespace utils::io {
void savePointCloud2AsPLY(const sensor_msgs::PointCloud2 &pcl,
                          const std::string &output_path,
                          int idx);

void savePointCloud2AsBinary(const sensor_msgs::PointCloud2 &pcl,
                             const std::string &output_path,
                             int idx);

void saveLaserScanAsPLY(const sensor_msgs::LaserScan &scan,
                        const std::string &output_path,
                        int idx);

void saveLaserScanAsBinary(const sensor_msgs::LaserScan &scan,
                           const std::string &output_path,
                           int idx);

void saveLaserScanAsBinary(const sensor_msgs::LaserScan &scan,
                           const std::string &output_path,
                           int idx);
}  // namespace utils::io
