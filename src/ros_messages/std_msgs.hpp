#pragma once

#include <fstream>

namespace std_msgs {

struct Time {
    int32_t secs;
    int32_t nsecs;
} __attribute__((aligned(8)));

struct Header {
    uint32_t seq;
    Time stamp;
    std::string frame_id;
} __attribute__((aligned(64))) __attribute__((packed));

Time parseTime(std::ifstream &rosbag, int &data_len);
Header parseHeader(std::ifstream &rosbag, int &data_len);
}  // namespace std_msgs