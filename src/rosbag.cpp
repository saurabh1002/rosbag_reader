#include "rosbag.hpp"

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>

using FieldValMap = std::map<std::string, std::shared_ptr<char[]>>;

Rosbag::Rosbag(const std::string &rosbag_path)
    : rosbag_path_(rosbag_path),
      rosbag_(rosbag_path, std::ios_base::in | std::ios_base::binary) {
    if (!rosbag_) {
        exit(EXIT_FAILURE);
    }

    std::string version;
    std::getline(rosbag_, version);
    if (version.find("V2.0") == std::string::npos) {
        exit(EXIT_FAILURE);
    }

    // Read Bag Header Record
    fields_ = this->readRecordHeader();
    index_pos_ = *reinterpret_cast<long int *>(fields_["index_pos"].get());
    num_unique_conns_ = *reinterpret_cast<int *>(fields_["conn_count"].get());
    num_chunk_records_ = *reinterpret_cast<int *>(fields_["chunk_count"].get());

    chunk_compression_types_.reserve(num_chunk_records_);
    uncompressed_chunk_sizes_.reserve(num_chunk_records_);

    // Ignore Data
    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
    rosbag_.ignore(data_len);
}

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
    std::map<std::string, std::shared_ptr<char[]>> fields;

    while (header_len != 0) {
        rosbag_.read(reinterpret_cast<char *>(&field_len), 4);
        std::getline(rosbag_, field_name, '=');
        const int field_val_nbytes =
                field_len - 1 - static_cast<int>(field_name.size());

        std::shared_ptr<char[]> buffer(new char[field_val_nbytes]);
        rosbag_.read(buffer.get(), field_val_nbytes);
        fields.insert({{field_name, std::move(buffer)}});
        header_len -= (4 + field_len);
    }
    return fields;
}

void Rosbag::readBag() {
    while (!rosbag_.eof()) {
        this->readRecord();
    }
}

void Rosbag::readRecord() {
    // Read Header
    fields_ = this->readRecordHeader();

    int opval = 0;
    if (fields_.find("op") != fields_.end()) {
        opval = int{*reinterpret_cast<uint8_t *>(fields_["op"].get())};
    }

    switch (opval) {
        case 5:
            this->readChunkRecord();
            break;

        case 7:
            this->readConnectionRecord();
            break;

        case 2:
            this->readMessageDataRecord();
            break;

        case 4:
            this->readIndexDataRecord();
            break;

        case 6:
            this->readChunkInfoRecord();
            break;

        default:
            break;
    }
}

int Rosbag::readChunkRecord() {
    chunk_compression_types_.emplace_back(fields_["compression"].get());
    uncompressed_chunk_sizes_.emplace_back(
            *reinterpret_cast<int *>(fields_["size"].get()));

    // Data
    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
    return data_len;
}

void Rosbag::readConnectionRecord() {
    auto topic = std::string(fields_["topic"].get());
    auto conn_id = *reinterpret_cast<int *>(fields_["conn"].get());

    // Data - Connection Header
    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);

    std::map<std::string, std::string> data_fields{
            {"topic", ""},    {"type", ""},
            {"md5sum", ""},   {"message_definition", ""},
            {"callerid", ""}, {"latching", ""}};

    while (data_len != 0) {
        auto [field_name, field_val] = this->readStringField(data_len);
        data_fields[field_name] = field_val;
    }
    connections_[conn_id] = {data_fields["type"], topic};
}

void Rosbag::readMessageDataRecord() {
    auto conn_id = *reinterpret_cast<int *>(fields_["conn"].get());
    auto time = *reinterpret_cast<long int *>(fields_["time"].get());

    // Data - Serialized Message Data
    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);

    long const buffer_offset = rosbag_.tellg();
    msgdata_info_.emplace_back(
            MesssageDataInfo{buffer_offset, data_len, conn_id, time});
    rosbag_.ignore(data_len);
}

void Rosbag::readIndexDataRecord() {
    auto conn = *reinterpret_cast<int *>(fields_["conn"].get());
    auto count = *reinterpret_cast<int *>(fields_["count"].get());
    auto ver = *reinterpret_cast<int *>(fields_["ver"].get());

    // Data
    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
    for (int i = 0; i < count; i++) {
        long int time = 0;
        rosbag_.read(reinterpret_cast<char *>(&time), sizeof(time));

        int offset = 0;
        rosbag_.read(reinterpret_cast<char *>(&offset), sizeof(offset));
    }
}

void Rosbag::readChunkInfoRecord() {
    auto ver = *reinterpret_cast<int *>(fields_["ver"].get());
    auto chunk_pos = *reinterpret_cast<long int *>(fields_["chunk_pos"].get());
    auto start_time =
            *reinterpret_cast<long int *>(fields_["start_time"].get());
    auto end_time = *reinterpret_cast<long int *>(fields_["end_time"].get());
    auto conn_count = *reinterpret_cast<int *>(fields_["count"].get());

    // Data
    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);

    for (int i = 0; i < conn_count; i++) {
        int conn = 0;
        rosbag_.read(reinterpret_cast<char *>(&conn), sizeof(conn));

        int msg_count = 0;
        rosbag_.read(reinterpret_cast<char *>(&msg_count), sizeof(msg_count));
    }
}

void Rosbag::info() const {
    std::cout << "Number of Chunk Records: " << num_chunk_records_ << "\n";
    std::cout << "Number of Unique Connections: " << num_unique_conns_ << "\n";

    std::cout << "Ros Topics in the Bag File\n";
    for (const auto &[key, value] : connections_) {
        const auto [msg_type, topic_name] = value;
        std::cout << "\tTopic Name: " << topic_name
                  << "\tMessage Type: " << msg_type << "\n";
    }
}