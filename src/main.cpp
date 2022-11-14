#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "indicators/progress_bar.hpp"
#include "utils.hpp"

int main(int argc, char* argv[]) {
    google::InitGoogleLogging("rosbag_reader");

    std::string const rosbag_path = argv[1];
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

    int chunk_count = 0;
    auto num_of_chunks = readRecord(rosbag, chunk_count);

    indicators::ProgressBar pbar{
            indicators::option::BarWidth{40},
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

        (void)readRecord(rosbag, chunk_count);

        pbar.set_option(indicators::option::PostfixText{
                std::to_string(chunk_count) + "/" +
                std::to_string(num_of_chunks)});
        pbar.set_progress(chunk_count);
    }

    pbar.set_option(
            indicators::option::PostfixText{"Done Reading the bag file!"});

    return 0;
}