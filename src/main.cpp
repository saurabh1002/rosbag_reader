#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "indicators/progress_bar.hpp"
#include "utils.hpp"

int main() {
    google::InitGoogleLogging("rosbag_reader");

    std::string const rosbag_path = "../pcl2.bag";
    std::ifstream rosbag(rosbag_path,
                         std::ios_base::in | std::ios_base::binary);
    if (!rosbag) {
        LOG(FATAL) << "Rosbag could not be opened, please check rosbag path";
        return 1;
    }

    std::string version;
    std::getline(rosbag, version);
    if (version.find("V2.0") == std::string::npos) {
        LOG(FATAL) << "Rosbag must be of version 2.0";
        return 1;
    }

    auto chunk_count = readRecord(rosbag);

    indicators::ProgressBar pbar{
            indicators::option::BarWidth{40},
            indicators::option::Start{"["},
            indicators::option::Fill{"■"},
            indicators::option::Lead{"■"},
            indicators::option::Remainder{"-"},
            indicators::option::End{"]"},
            indicators::option::MaxProgress{chunk_count},
            indicators::option::ForegroundColor{indicators::Color::cyan},
            indicators::option::PostfixText{"Reading PointClouds from rosbag"},
            indicators::option::ShowElapsedTime{true},
            indicators::option::ShowRemainingTime{true},
            indicators::option::FontStyles{std::vector<indicators::FontStyle>{
                    indicators::FontStyle::bold}}};

    for (int idx = 0; idx < chunk_count; idx++) {
        auto uncompressed_chunk_size = readRecord(rosbag);
        (void)readRecord(rosbag);

        pbar.set_option(indicators::option::PostfixText{
                std::to_string(idx + 1) + "/" + std::to_string(chunk_count)});
        pbar.tick();
    }
    pbar.set_option(
            indicators::option::PostfixText{"Done Reading the bag file!"});

    return 0;
}