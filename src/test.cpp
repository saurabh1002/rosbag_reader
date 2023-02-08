#include <string>

#include "rosbag.hpp"
#include "sensor_msgs.hpp"
#include "utils.hpp"

int main(int argc, char* argv[]) {
    const std::string rosbag_path(argv[1]);
    const std::string topic_name(argv[2]);
    const std::string ply_save_path(argv[3]);

    Rosbag rosbag(rosbag_path);
    rosbag.printInfo();
    rosbag.readData();

    auto pointclouds = rosbag.extractPointCloud2(topic_name);
    int idx = 0;
    for (auto& pcl : pointclouds) {
        utils::io::savePointCloudAsPLY(pcl, ply_save_path + topic_name, idx++);
    }

    return 0;
}