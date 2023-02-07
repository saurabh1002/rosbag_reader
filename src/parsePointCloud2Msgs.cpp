#include <fstream>
#include <string>

#include "rosbag.hpp"
#include "sensor_msgs.hpp"
#include "utils.hpp"

int main(int argc, char* argv[]) {
    const std::string rosbag_path(argv[1]);
    const std::string ply_save_path(argv[2]);

    Rosbag rosbag(rosbag_path);
    rosbag.printInfo();
    rosbag.readData();

    auto connections = rosbag.getConnections();

    std::ifstream bag(rosbag_path, std::ios_base::in | std::ios_base::binary);

    for (auto& [conn_id, conn_info] : connections) {
        if (conn_info.msg_type == "sensor_msgs/PointCloud2") {
            int idx = 0;
            for (auto& msg_info : conn_info.messages_info) {
                bag.seekg(msg_info.buffer_offset);
                auto pcl = sensor_msgs::parsePointCloud2(bag);
                utils::io::savePointCloudAsPLY(
                        pcl, ply_save_path + conn_info.topic_name, idx++);
            }
        }
    }
    return 0;
}
