#include "header.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include "utils.hpp"

void timestamp::read(std::ifstream &rosbag, int &data_len) {
    std::printf("\tstamp:\n");
    int32_t secs = 0;
    int32_t nsecs = 0;
    rosbag.read(reinterpret_cast<char *>(&secs), sizeof(secs));
    rosbag.read(reinterpret_cast<char *>(&nsecs), sizeof(nsecs));
    std::printf("\t\tsecs: %d\n", secs);
    std::printf("\t\tnsecs: %d\n", nsecs);
    data_len -= static_cast<int>(sizeof(secs) + sizeof(nsecs));
}

void header::read(std::ifstream &rosbag, int &data_len) {
    std::printf("header:\n");
    uint32_t seq = 0;
    rosbag.read(reinterpret_cast<char *>(&seq), sizeof(seq));
    std::printf("\tseq: %u\n", seq);

    timestamp::read(rosbag, data_len);

    std::string frame_id;
    int32_t len_frame_id = 0;
    rosbag.read(reinterpret_cast<char *>(&len_frame_id), sizeof(len_frame_id));
    readString(rosbag, frame_id, len_frame_id);
    std::cout << "\tframe_id: " << frame_id << "\n";

    data_len -=
            static_cast<int>(sizeof(seq) + sizeof(len_frame_id) + len_frame_id);
}