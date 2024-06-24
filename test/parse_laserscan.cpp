#include <string>

#include "rosbag.h"
#include "utils.h"

int main(int argc, char* argv[]) {
    const std::string rosbag_path(argv[1]);
    const std::string topic_name(argv[2]);
    const std::string ply_save_path(argv[3]);

    Rosbag rosbag(rosbag_path);
    rosbag.printInfo();
    rosbag.readData();

    auto num_of_messages = rosbag.getNumMsgsonTopic(topic_name);
    for (int idx = 0; idx < num_of_messages; idx++) {
        auto scan = rosbag.extractLaserScan(topic_name, idx);
        utils::io::saveLaserScanAsPLY(scan, ply_save_path, idx);
    }
    return 0;
}
