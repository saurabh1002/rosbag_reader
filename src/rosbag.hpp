#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

using Connection = std::map<int, std::tuple<std::string, std::string>>;
using FieldValMap = std::map<std::string, std::shared_ptr<char[]>>;

struct MesssageDataInfo {
    long buffer_offset;
    int data_len;
    int conn_id;
    long int time;
} __attribute__((aligned(32)));

class Rosbag {
public:
    Rosbag(const std::string &rosbag_path);

private:
    void readString(std::string &str, int n_bytes);
    std::tuple<std::string, std::string> readStringField(int &header_len);

public:
    FieldValMap readRecordHeader();
    void readBag();
    void readRecord();
    int readChunkRecord();
    void readConnectionRecord();
    void readMessageDataRecord();
    void readIndexDataRecord();
    void readChunkInfoRecord();

public:
    inline std::vector<MesssageDataInfo> getMessageDataInfo() const {
        return msgdata_info_;
    }
    inline Connection getConnections() const { return connections_; }
    void info() const;

private:
    std::string rosbag_path_;
    std::ifstream rosbag_;

    long int index_pos_;
    int num_unique_conns_;
    int num_chunk_records_;

    std::map<std::string, std::shared_ptr<char[]>> fields_;

    std::vector<std::string> chunk_compression_types_;
    std::vector<int> uncompressed_chunk_sizes_;
    Connection connections_;

    std::vector<MesssageDataInfo> msgdata_info_;
};