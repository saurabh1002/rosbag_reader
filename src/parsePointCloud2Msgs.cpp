#include <glog/logging.h>

#include <fstream>
#include <string>
#include <tuple>

#include "indicators/progress_bar.hpp"
#include "rosbag.hpp"
#include "yaml-cpp/yaml.h"

int main(int argc, char *argv[]) {
    google::InitGoogleLogging("rosbag_reader");

    YAML::Node config = YAML::LoadFile(argv[1]);

    auto rosbag_path = config["rosbag_path"].as<std::string>();
    std::ifstream rosbag(rosbag_path,
                         std::ios_base::in | std::ios_base::binary);

    auto pcl_save_path = config["pcl_save_path"].as<std::string>();

    if (!rosbag) {
        return 1;
    }

    std::string version;
    std::getline(rosbag, version);
    if (version.find("V2.0") == std::string::npos) {
        return 1;
    }

    // Read Bag Header Record
    int header_len = 0;
    rosbag.read(reinterpret_cast<char *>(&header_len), 4);

    int chunk_count = 0;
    auto fields = readRecordHeader(rosbag, header_len);
    auto [index_pos, conn_count, num_of_chunks] =
            readBagHeaderRecord(rosbag, fields);

    std::vector<std::tuple<std::string, std::string>> connections;
    connections.reserve(conn_count);

    while (true) {
        if (rosbag.eof()) {
            break;
        }
        readRecord(rosbag, chunk_count, connections, pcl_save_path);
    }

    return 0;
}
