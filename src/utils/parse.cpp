#include "parse.hpp"

#include <glog/logging.h>

#include <fstream>
#include <memory>
#include <string>
#include <tuple>

void readString(std::ifstream &rosbag, std::string &str, const int n_bytes) {
    std::unique_ptr<char[]> const buffer(new char[n_bytes + 1]);
    buffer.get()[n_bytes] = '\0';
    rosbag.read(buffer.get(), n_bytes);
    str = buffer.get();
}

std::tuple<std::string, std::string> readStringField(std::ifstream &rosbag,
                                                     int &header_len) {
    int field_len = 0;
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    std::string field_name;
    std::getline(rosbag, field_name, '=');
    std::string field_val;
    readString(rosbag, field_val,
               field_len - static_cast<int>(field_name.size()) - 1);
    DLOG(INFO) << field_name << " = " << field_val;
    header_len -= (field_len + 4);

    return std::make_tuple(field_name, field_val);
}
