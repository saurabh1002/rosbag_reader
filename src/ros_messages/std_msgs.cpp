#include "std_msgs.hpp"

#include <glog/logging.h>

#include <fstream>
#include <string>

#include "parse.hpp"

void parseHeaderMsg(std::ifstream &rosbag, int &data_len) {
    DLOG(INFO) << "header:";
    uint32_t seq = 0;
    rosbag.read(reinterpret_cast<char *>(&seq), sizeof(seq));

    DLOG(INFO) << "\tseq: " << seq;

    DLOG(INFO) << "\tstamp:";
    int32_t secs = 0;
    int32_t nsecs = 0;
    rosbag.read(reinterpret_cast<char *>(&secs), sizeof(secs));
    rosbag.read(reinterpret_cast<char *>(&nsecs), sizeof(nsecs));
    DLOG(INFO) << "\t\tsecs: ", secs;
    DLOG(INFO) << "\t\tnsecs: ", nsecs;
    data_len -= static_cast<int>(sizeof(secs) + sizeof(nsecs));

    std::string frame_id;
    int32_t len_frame_id = 0;
    rosbag.read(reinterpret_cast<char *>(&len_frame_id), sizeof(len_frame_id));
    readString(rosbag, frame_id, len_frame_id);
    DLOG(INFO) << "\tframe_id: " << frame_id;

    data_len -=
            static_cast<int>(sizeof(seq) + sizeof(len_frame_id) + len_frame_id);
}