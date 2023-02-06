#include "std_msgs.hpp"

#include <fstream>
#include <string>

#include "utils.hpp"

std_msgs::Time std_msgs::parseTime(std::ifstream &rosbag, int &data_len) {
    int32_t secs = 0;
    int32_t nsecs = 0;
    rosbag.read(reinterpret_cast<char *>(&secs), sizeof(secs));
    rosbag.read(reinterpret_cast<char *>(&nsecs), sizeof(nsecs));
    data_len -= 8;

    return std_msgs::Time{secs, nsecs};
}

std_msgs::Header std_msgs::parseHeader(std::ifstream &rosbag, int &data_len) {
    uint32_t seq = 0;
    rosbag.read(reinterpret_cast<char *>(&seq), sizeof(seq));

    auto stamp = std_msgs::parseTime(rosbag, data_len);

    std::string frame_id;
    int32_t len_frame_id = 0;
    rosbag.read(reinterpret_cast<char *>(&len_frame_id), sizeof(len_frame_id));
    utils::parser::readString(rosbag, frame_id, len_frame_id);

    data_len -= (8 + len_frame_id);
    return std_msgs::Header{seq, stamp, frame_id};
}