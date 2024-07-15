#include <fstream>
#include <memory>
#include <string>

#include "ros_messages.h"

std::string std_msgs::readString(std::ifstream &rosbag, const int n_bytes) {
    std::unique_ptr<char[]> const buffer(new char[n_bytes + 1]);
    buffer.get()[n_bytes] = '\0';
    rosbag.read(buffer.get(), n_bytes);
    return buffer.get();
}

std_msgs::Time std_msgs::parseTime(std::ifstream &rosbag) {
    int32_t secs = 0;
    int32_t nsecs = 0;
    rosbag.read(reinterpret_cast<char *>(&secs), sizeof(secs));
    rosbag.read(reinterpret_cast<char *>(&nsecs), sizeof(nsecs));

    return std_msgs::Time{secs, nsecs};
}

std_msgs::Header std_msgs::parseHeader(std::ifstream &rosbag) {
    uint32_t seq = 0;
    rosbag.read(reinterpret_cast<char *>(&seq), sizeof(seq));

    auto stamp = std_msgs::parseTime(rosbag);

    int32_t len_frame_id = 0;
    rosbag.read(reinterpret_cast<char *>(&len_frame_id), sizeof(len_frame_id));
    std::string const frame_id = readString(rosbag, len_frame_id);

    return std_msgs::Header{seq, stamp, frame_id};
}
