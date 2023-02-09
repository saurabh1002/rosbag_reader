#pragma once

#include <fstream>
#include <string>

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

std::string readString(std::ifstream &rosbag, int n_bytes);
Time parseTime(std::ifstream &rosbag);
Header parseHeader(std::ifstream &rosbag);
}  // namespace std_msgs