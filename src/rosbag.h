#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ros_messages.h"

struct ChunkInfo {
    long int chunk_pos{};
    long int start_time{};
    long int end_time{};
    long int num_msgs{};
    int conn_count{};
} __attribute__((aligned(64)));

struct MesssageDataInfo {
    long buffer_offset{};
    int conn_id{};
    long int time{};
} __attribute__((aligned(32)));

struct ConnectionInfo {
    std::string msg_type;
    std::string topic_name;
    int num_msgs{};
    std::vector<MesssageDataInfo> messages_info;
} __attribute__((aligned(128))) __attribute__((packed));

class Rosbag {
    using Connection = std::map<int, ConnectionInfo>;
    using FieldValMap = std::map<std::string, std::shared_ptr<char[]>>;

public:
    Rosbag(const std::string &rosbag_path);

public:
    void readString(std::string &str, int n_bytes);
    std::tuple<std::string, std::string> readStringField(int &header_len);
    FieldValMap readRecordHeader();
    void readBagHeaderRecord();
    void readChunkRecord(long int num_of_msgs);
    void readConnectionRecord();
    void readMessageDataRecord();
    void readChunkInfoRecord();

public:
    void readData();
    void readBagInfo();
    void printInfo() const;
    inline Connection getConnections() const { return connections_; }
    inline int getNumMsgsonTopic(const std::string &topic) {
        return connections_[topic_to_conn_id_[topic]].num_msgs;
    }

public:
    sensor_msgs::PointCloud2 extractPointCloud2(const std::string &topic_name,
                                                int msg_idx);
    sensor_msgs::LaserScan extractLaserScan(const std::string &topic_name,
                                                int msg_idx);

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