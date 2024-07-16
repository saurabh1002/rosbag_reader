// MIT License

// Copyright (c) 2024 Saurabh Gupta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "messages/messages.h"

struct ChunkInfo {
    long int chunk_pos{};
    long int start_time{};
    long int end_time{};
    long int num_msgs{};
    int conn_count{};
};

struct MesssageDataInfo {
    long buffer_offset{};
    int conn_id{};
    long int time{};
};

struct ConnectionInfo {
    std::string msg_type;
    std::string topic_name;
    int num_msgs{};
    std::vector<MesssageDataInfo> messages_info;
};

class Rosbag {
    using Connection = std::map<int, ConnectionInfo>;
    using FieldValMap = std::map<std::string, std::shared_ptr<char[]>>;

public:
    Rosbag(const std::string &rosbag_path);

private:
    void readString(std::string &str, int n_bytes);
    std::tuple<std::string, std::string> readStringField(int &header_len);
    FieldValMap readRecordHeader();
    void readBagHeaderRecord();
    void readChunkRecord(long int num_of_msgs);
    void readConnectionRecord();
    void readMessageDataRecord();
    void readChunkInfoRecord();

    void readBagInfo();
    inline Connection getConnections() const { return connections_; }

public:
    inline int getNumMsgsonTopic(const std::string &topic) const {
        return connections_.at(topic_to_conn_id_.at(topic)).num_msgs;
    }
    void readData();
    void printInfo() const;
    void printAvailableTopics() const;
    void savePointCloud2AsPLY(const std::string &topic_name,
                              const std::string &output_path);
    void saveLaserScanAsPLY(const std::string &topic_name,
                            const std::string &output_path);

private:
    std::string rosbag_path_;
    std::ifstream rosbag_;

    std::string version_;

    long int index_pos_{};
    int num_unique_conns_{};
    int num_chunk_records_{};

    std::map<std::string, std::shared_ptr<char[]>> fields_;

    Connection connections_;
    std::map<std::string, int> topic_to_conn_id_;
    std::vector<std::string> chunk_compression_types_;
    std::vector<ChunkInfo> chunk_info_records_;
};
