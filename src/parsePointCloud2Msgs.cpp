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
        LOG(FATAL) << "Rosbag could not be opened, please check rosbag path: ";
        return 1;
    }

    std::string version;
    std::getline(rosbag, version);
    if (version.find("V2.0") == std::string::npos) {
        LOG(FATAL) << "Rosbag must be of version 2.0";
        return 1;
    }

    // Read Bag Header Record
    int header_len = 0;
    rosbag.read(reinterpret_cast<char *>(&header_len), 4);
    DLOG(INFO) << "Record Header Length = " << header_len << std::endl;

    int chunk_count = 0;
    auto fields = readRecordHeader(rosbag, header_len);
    LOG_IF(WARNING, fields.find("op") == fields.end())
            << "OP Code not found in bag header record header";
    auto [index_pos, conn_count, num_of_chunks] =
            readBagHeaderRecord(rosbag, fields);

    std::vector<std::tuple<std::string, std::string>> connections;
    connections.reserve(conn_count);

    indicators::ProgressBar pbar{
            indicators::option::BarWidth{50},
            indicators::option::Start{"["},
            indicators::option::Fill{"■"},
            indicators::option::Lead{"■"},
            indicators::option::Remainder{"-"},
            indicators::option::End{"]"},
            indicators::option::MaxProgress{num_of_chunks},
            indicators::option::ForegroundColor{indicators::Color::cyan},
            indicators::option::PostfixText{"Reading PointClouds from rosbag"},
            indicators::option::ShowElapsedTime{true},
            indicators::option::ShowRemainingTime{true},
            indicators::option::FontStyles{std::vector<indicators::FontStyle>{
                    indicators::FontStyle::bold}}};

    while (true) {
        if (rosbag.eof()) {
            LOG(WARNING) << "End of File reached";
            break;
        }

        readRecord(rosbag, chunk_count, connections, pcl_save_path);

        pbar.set_option(indicators::option::PostfixText{
                std::to_string(chunk_count) + "/" +
                std::to_string(num_of_chunks)});
        pbar.set_progress(chunk_count);
    }

    pbar.set_option(
            indicators::option::PostfixText{"Done Reading the bag file!"});

    return 0;
}