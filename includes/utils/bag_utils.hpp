#pragma once

#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <tuple>

void readString(std::ifstream &rosbag, std::string &str, int n_bytes);

std::tuple<std::string, std::string> readStringField(std::ifstream &rosbag,
                                                     int &header_len);

template <typename T>
std::tuple<std::string, T> readField(std::ifstream &rosbag, int &header_len) {
    std::string field_name;
    int field_len = 0;
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);

    T field_val;
    std::getline(rosbag, field_name, '=');
    rosbag.read(reinterpret_cast<char *>(&field_val), sizeof(field_val));
    DLOG(INFO) << field_name.c_str() << " = " << field_val;

    header_len -= (4 + field_len);

    return std::make_tuple(field_name, field_val);
}

std::map<std::string, std::shared_ptr<char[]>> readRecordHeader(
        std::ifstream &rosbag, int &header_len);

void readRecord(std::ifstream &rosbag,
                int &chunk_count,
                std::string &msg_type,
                std::vector<std::tuple<std::string, std::string>> &connections);

std::tuple<long int, int, int> readBagHeader(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> fields);

void readChunk(std::ifstream &rosbag,
               std::map<std::string, std::shared_ptr<char[]>> fields,
               int &chunk_count);

void readConnection(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> fields,
        std::string &msg_type,
        std::vector<std::tuple<std::string, std::string>> &connections);

void readMessageData(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> fields,
        std::string &msg_type,
        std::vector<std::tuple<std::string, std::string>> &connections);

void readIndexData(std::ifstream &rosbag,
                   std::map<std::string, std::shared_ptr<char[]>> fields);

void readChunkInfo(std::ifstream &rosbag,
                   std::map<std::string, std::shared_ptr<char[]>> fields);
