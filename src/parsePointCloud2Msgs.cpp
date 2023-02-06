#include <fstream>
#include <string>

#include "rosbag.hpp"
#include "sensor_msgs.hpp"
#include "utils.hpp"

int main(int argc, char *argv[]) {
    const std::string rosbag_path(argv[1]);
    const std::string ply_save_path(argv[2]);

    Rosbag rosbag(rosbag_path);
    rosbag.readBag();

    auto message_data_info = rosbag.getMessageDataInfo();
    auto connections = rosbag.getConnections();
    rosbag.info();

    std::ifstream bag(rosbag_path, std::ios_base::in | std::ios_base::binary);
    int i = 0;
    for (auto message_data : message_data_info) {
        bag.ignore(message_data.buffer_offset);
        auto [msg_type, topic] = connections[message_data.conn_id];

        if (msg_type == "sensor_msgs/PointCloud2") {
            auto pcl =
                    sensor_msgs::parsePointCloud2(bag, message_data.data_len);
            utils::io::savePointCloudAsPLY(pcl, ply_save_path + topic, i);
            i++;
        }
        bag.clear();
        bag.seekg(0, std::ios::beg);
    }

    return 0;
}
