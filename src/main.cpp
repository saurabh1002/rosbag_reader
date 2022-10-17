#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "utils.hpp"

std::map<std::string, int> fields_map{
        {"op", 0}, {"index_pos", 1}, {"conn_count", 2}, {"chunk_count", 3}};

int main() {
    std::string const rosbag_path = "../pcl2.bag";
    std::ifstream rosbag(rosbag_path,
                         std::ios_base::in | std::ios_base::binary);
    if (!rosbag) {
        std::cerr << "Rosbag could not be opened, please check rosbag path"
                  << std::endl;
        return 1;
    }

    std::string version;
    std::getline(rosbag, version);
    if (version.find("V2.0") == std::string::npos) {
        std::cerr << "Rosbag must be of version 2.0" << std::endl;
        return 1;
    }

    auto chunk_count = readRecord(rosbag);

    auto uncompressed_chunk_size = readRecord(rosbag);
}