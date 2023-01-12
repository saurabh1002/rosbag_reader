#include <fstream>
#include <string>
#include <tuple>

#include "rosbag.hpp"

int main(int argc, char *argv[]) {
    std::string rosbag_path(argv[1]);
    std::string ply_save_path(argv[2]);

    std::ifstream rosbag(rosbag_path,
                         std::ios_base::in | std::ios_base::binary);

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
        readRecord(rosbag, chunk_count, connections, ply_save_path);
    }

    return 0;
}
