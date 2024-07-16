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
#include "rosbag.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <utility>

#include "messages/messages.h"

using FieldValMap =
        std::map<std::string, std::pair<std::shared_ptr<char[]>, int>>;

void Rosbag::readString(std::string &str, const int n_bytes) {
    std::unique_ptr<char[]> const buffer(new char[n_bytes + 1]);
    buffer.get()[n_bytes] = '\0';
    rosbag_.read(buffer.get(), n_bytes);
    str = buffer.get();
}

std::tuple<std::string, std::string> Rosbag::readStringField(int &header_len) {
    int field_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&field_len), 4);
    std::string field_name;
    std::getline(rosbag_, field_name, '=');
    std::string field_val;
    this->readString(field_val,
                     field_len - static_cast<int>(field_name.size()) - 1);
    header_len -= (field_len + 4);
    return {field_name, field_val};
}

FieldValMap Rosbag::readRecordHeader() {
    int header_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&header_len), 4);

    int field_len = 0;
    std::string field_name;
    FieldValMap fields;

    while (header_len != 0) {
        rosbag_.read(reinterpret_cast<char *>(&field_len), 4);
        std::getline(rosbag_, field_name, '=');
        const int field_val_nbytes =
                field_len - 1 - static_cast<int>(field_name.size());

        std::shared_ptr<char[]> buffer(new char[field_val_nbytes]);
        rosbag_.read(buffer.get(), field_val_nbytes);
        fields.insert({{field_name, std::make_pair(buffer, field_val_nbytes)}});
        header_len -= (4 + field_len);
    }
    return fields;
}

Rosbag::Rosbag(const std::string &rosbag_path)
    : rosbag_path_(rosbag_path),
      rosbag_(rosbag_path, std::ios_base::in | std::ios_base::binary) {
    if (!rosbag_) {
        std::cerr << "Invalid path to the bag file" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::getline(rosbag_, version_);
    if (version_.find("V2.0") != std::string::npos) {
        version_ = "2.0";
    } else {
        std::cerr << "Supports only Rosbag Version 2.0" << std::endl;
        exit(EXIT_FAILURE);
    }

    this->readBagInfo();

    for (auto &[conn_id, conn_info] : connections_) {
        conn_info.messages_info.reserve(conn_info.num_msgs);
    }
}

void Rosbag::readBagHeaderRecord() {
    fields_ = this->readRecordHeader();
    index_pos_ =
            *reinterpret_cast<long int *>(fields_["index_pos"].first.get());
    num_unique_conns_ =
            *reinterpret_cast<int *>(fields_["conn_count"].first.get());
    num_chunk_records_ =
            *reinterpret_cast<int *>(fields_["chunk_count"].first.get());

    chunk_info_records_.reserve(num_chunk_records_);
    chunk_compression_types_.reserve(num_chunk_records_);

    // Ignore Data
    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
    rosbag_.ignore(data_len);
}

void Rosbag::readBagInfo() {
    this->readBagHeaderRecord();

    rosbag_.seekg(index_pos_);
    for (int i = 0; i < num_unique_conns_; i++) {
        this->readConnectionRecord();
    }
    for (int i = 0; i < num_chunk_records_; i++) {
        this->readChunkInfoRecord();
    }
}

void Rosbag::readData() {
    std::for_each(chunk_info_records_.cbegin(), chunk_info_records_.cend(),
                  [&](const ChunkInfo chunk_info) {
                      rosbag_.seekg(chunk_info.chunk_pos);
                      this->readChunkRecord(chunk_info.num_msgs);
                  });
}

void Rosbag::readChunkRecord(long int num_of_msgs) {
    fields_ = this->readRecordHeader();
    int opval = int{*reinterpret_cast<uint8_t *>(fields_["op"].first.get())};
    if (opval == 5) {
        chunk_compression_types_.emplace_back(
                fields_["compression"].first.get());
        int data_len = 0;
        rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
        while (num_of_msgs != 0) {
            fields_ = this->readRecordHeader();
            opval = int{
                    *reinterpret_cast<uint8_t *>(fields_["op"].first.get())};
            if (opval == 2) {
                this->readMessageDataRecord();
                num_of_msgs--;
            } else {
                int data_len = 0;
                rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
                rosbag_.ignore(data_len);
            }
        }
        return;
    }
    std::cerr << "Expected to Read Chunk Record" << std::endl;
}

void Rosbag::readConnectionRecord() {
    fields_ = this->readRecordHeader();
    int const opval =
            int{*reinterpret_cast<uint8_t *>(fields_["op"].first.get())};
    if (opval == 7) {
        ConnectionInfo info{};
        info.topic_name = std::string(fields_["topic"].first.get(),
                                      fields_["topic"].second);
        auto conn_id = *reinterpret_cast<int *>(fields_["conn"].first.get());

        // Data - Connection Header
        int data_len = 0;
        rosbag_.read(reinterpret_cast<char *>(&data_len), 4);

        std::map<std::string, std::string> data_fields;
        while (data_len != 0) {
            auto [field_name, field_val] = this->readStringField(data_len);
            data_fields[field_name] = field_val;
        }

        info.msg_type = data_fields["type"];
        topic_to_conn_id_[info.topic_name] = conn_id;
        connections_[conn_id] = info;
        return;
    }
    std::cerr << "Expected to Read Connection Record" << std::endl;
}

void Rosbag::readMessageDataRecord() {
    auto conn_id = *reinterpret_cast<int *>(fields_["conn"].first.get());
    auto time = *reinterpret_cast<long int *>(fields_["time"].first.get());

    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
    connections_[conn_id].messages_info.emplace_back(
            MesssageDataInfo{rosbag_.tellg(), conn_id, time});
    rosbag_.ignore(data_len);
}

void Rosbag::readChunkInfoRecord() {
    fields_ = this->readRecordHeader();
    int const opval =
            int{*reinterpret_cast<uint8_t *>(fields_["op"].first.get())};
    if (opval == 6) {
        ChunkInfo info{};
        info.chunk_pos =
                *reinterpret_cast<long int *>(fields_["chunk_pos"].first.get());
        info.start_time = *reinterpret_cast<long int *>(
                fields_["start_time"].first.get());
        info.end_time =
                *reinterpret_cast<long int *>(fields_["end_time"].first.get());
        info.conn_count =
                *reinterpret_cast<int *>(fields_["count"].first.get());

        // Data
        int data_len = 0;
        rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
        int conn = 0;
        int count = 0;
        for (int i = 0; i < info.conn_count; i++) {
            rosbag_.read(reinterpret_cast<char *>(&conn), 4);
            rosbag_.read(reinterpret_cast<char *>(&count), 4);
            connections_[conn].num_msgs += count;
            info.num_msgs += count;
        }
        chunk_info_records_.emplace_back(info);
        return;
    }
    std::cerr << "Expected to Read Chunk Info Record" << std::endl;
}

void Rosbag::printInfo() const {
    std::cout << "path:\t\t\t" << rosbag_path_ << std::endl;
    std::cout << "version:\t\t" << version_ << std::endl;

    auto start = std::min_element(chunk_info_records_.cbegin(),
                                  chunk_info_records_.cend(),
                                  [](const auto &rec1, const auto &rec2) {
                                      return rec1.start_time < rec2.start_time;
                                  });

    auto end = std::max_element(chunk_info_records_.cbegin(),
                                chunk_info_records_.cend(),
                                [](const auto &rec1, const auto &rec2) {
                                    return rec1.end_time < rec2.end_time;
                                });

    std::cout << "duration:\t\t" << end->end_time - start->start_time
              << std::endl;
    std::cout << "start:\t\t\t" << start->start_time << std::endl;
    std::cout << "end:\t\t\t" << end->end_time << std::endl;

    auto num_of_messages = std::accumulate(
            chunk_info_records_.cbegin(), chunk_info_records_.cend(), 0,
            [](long int sum, ChunkInfo info) { return sum + info.num_msgs; });
    std::cout << "messages:\t\t" << num_of_messages << std::endl;

    std::cout << "topics:" << std::endl;
    std::for_each(
            connections_.cbegin(), connections_.cend(),
            [](const auto connection) {
                std::cout << "\t" << connection.second.topic_name << std::endl;
                std::cout << "\t\t" << connection.second.num_msgs << " msgs"
                          << std::endl;
                std::cout << "\t\tmsg type: " << connection.second.msg_type
                          << std::endl;
            });
}

void Rosbag::printAvailableTopics() const {
    std::cout << "topics:" << std::endl;
    std::for_each(connections_.cbegin(), connections_.cend(),
                  [](const auto connection) {
                      std::cout << "\t" << connection.second.topic_name
                                << std::endl;
                  });
}

void Rosbag::saveDataOnTopic(const std::string &topic_name,
                             const std::string &output_path) {
    if (topic_to_conn_id_.find(topic_name) != topic_to_conn_id_.cend()) {
        auto conn_id = topic_to_conn_id_.at(topic_name);
        auto conn_info = connections_.at(conn_id);
        if (conn_info.msg_type == "sensor_msgs/PointCloud2") {
            Rosbag::readData();
            Rosbag::savePointCloud2AsPLY(conn_info.messages_info, output_path);
        } else if (conn_info.msg_type == "sensor_msgs/LaserScan") {
            Rosbag::readData();
            Rosbag::saveLaserScanAsPLY(conn_info.messages_info, output_path);
        } else {
            std::cerr << "Message Type of topic " << topic_name
                      << " not supported" << std::endl;
            std::exit(1);
        }
    } else {
        std::cerr << "Topic " << topic_name << " not found in the bagfile"
                  << std::endl;
        std::exit(1);
    }
}

void Rosbag::savePointCloud2AsPLY(
        const std::vector<MesssageDataInfo> &messages_info,
        const std::string &output_path) {
    std::for_each(messages_info.cbegin(), messages_info.cend(),
                  [&](const auto &msg_info) {
                      rosbag_.seekg(msg_info.buffer_offset);
                      auto pointcloud = sensor_msgs::PointCloud2(rosbag_);
                      pointcloud.saveAsPLY(output_path);
                  });
}

void Rosbag::saveLaserScanAsPLY(
        const std::vector<MesssageDataInfo> &messages_info,
        const std::string &output_path) {
    std::for_each(messages_info.cbegin(), messages_info.cend(),
                  [&](const auto &msg_info) {
                      rosbag_.seekg(msg_info.buffer_offset);
                      auto laserscan = sensor_msgs::LaserScan(rosbag_);
                      laserscan.saveAsPLY(output_path);
                  });
}
