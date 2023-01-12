#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

std::map<std::string, std::shared_ptr<char[]>> readRecordHeader(
        std::ifstream &rosbag, int &header_len);

void readRecord(std::ifstream &rosbag,
                int &chunk_count,
                std::vector<std::tuple<std::string, std::string>> &connections,
                const std::string &pcl_save_path);

std::tuple<long int, int, int> readBagHeaderRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields);

void readChunkRecord(std::ifstream &rosbag,
                     std::map<std::string, std::shared_ptr<char[]>> &fields,
                     int &chunk_count);

void readConnectionRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields,
        std::vector<std::tuple<std::string, std::string>> &connections);

void readMessageDataRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields,
        const std::vector<std::tuple<std::string, std::string>> &connections,
        const std::string &pcl_save_path);

void readIndexDataRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields);

void readChunkInfoRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields);
