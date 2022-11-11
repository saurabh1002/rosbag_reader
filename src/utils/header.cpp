#include "header.hpp"

#include <glog/logging.h>

#include <fstream>
#include <string>

#include "utils.hpp"

void timestamp::read(std::ifstream &rosbag, int &data_len) {
    LOG(INFO) << "\tstamp:";
    int32_t secs = 0;
    int32_t nsecs = 0;
    rosbag.read(reinterpret_cast<char *>(&secs), sizeof(secs));
    rosbag.read(reinterpret_cast<char *>(&nsecs), sizeof(nsecs));
    LOG(INFO) << "\t\tsecs: ", secs;
    LOG(INFO) << "\t\tnsecs: ", nsecs;
    data_len -= static_cast<int>(sizeof(secs) + sizeof(nsecs));
}

void header::read(std::ifstream &rosbag, int &data_len) {
    LOG(INFO) << "header:";
    uint32_t seq = 0;
    rosbag.read(reinterpret_cast<char *>(&seq), sizeof(seq));

    LOG(INFO) << "\tseq: " << seq;

    timestamp::read(rosbag, data_len);

    std::string frame_id;
    int32_t len_frame_id = 0;
    rosbag.read(reinterpret_cast<char *>(&len_frame_id), sizeof(len_frame_id));
    readString(rosbag, frame_id, len_frame_id);
    LOG(INFO) << "\tframe_id: " << frame_id;

    data_len -=
            static_cast<int>(sizeof(seq) + sizeof(len_frame_id) + len_frame_id);
}