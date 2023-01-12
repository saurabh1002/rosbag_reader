#include "std_msgs.hpp"

#include <fstream>
#include <string>

#include "utils/parse.hpp"

void parseHeaderMsg(std::ifstream &rosbag, int &data_len) {
    uint32_t seq = 0;
    rosbag.read(reinterpret_cast<char *>(&seq), sizeof(seq));

    int32_t secs = 0;
    int32_t nsecs = 0;
    rosbag.read(reinterpret_cast<char *>(&secs), sizeof(secs));
    rosbag.read(reinterpret_cast<char *>(&nsecs), sizeof(nsecs));
    data_len -= int{sizeof(secs) + sizeof(nsecs)};

    std::string frame_id;
    int32_t len_frame_id = 0;
    rosbag.read(reinterpret_cast<char *>(&len_frame_id), sizeof(len_frame_id));
    readString(rosbag, frame_id, len_frame_id);

    data_len -= (int{sizeof(seq) + sizeof(len_frame_id)} + len_frame_id);
}
