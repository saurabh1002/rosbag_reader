#include "utils.hpp"

#include <glog/logging.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

void readString(std::ifstream &rosbag, std::string &str, int n_bytes) {
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
    LOG(INFO) << field_name << " = " << field_val;
    header_len -= (field_len + 4);

    return std::make_tuple(field_name, field_val);
}

int readRecord(std::ifstream &rosbag) {
    // Read Header Length
    int header_len = 0;
    rosbag.read(reinterpret_cast<char *>(&header_len), 4);
    LOG(INFO) << "Record Header Length = " << header_len << std::endl;

    // Read Header
    int retval = 0;
    int opval = 0;

    // Read op field
    opval = readOp(rosbag, header_len);

    switch (opval) {
        case 3: {
            auto [index_pos, conn_count, chunk_count] =
                    readBagHeader(rosbag, header_len);
            retval = chunk_count;
        } break;

        case 5:
            retval = readChunk(rosbag, header_len);
            break;

        case 7:
            readConnection(rosbag, header_len);
            break;

        case 2:
            readMessageData(rosbag, header_len);
            break;

        case 4:
            readIndexData(rosbag, header_len);
            break;

        case 6:
            readChunkInfo(rosbag, header_len);
            break;

        default:
            retval = 0;
            LOG(WARNING) << "No record reader for opval = " << opval;
            break;
    }
    return retval;
}

int readOp(std::ifstream &rosbag, int &header_len) {
    std::string field_name;
    int field_len = 0;
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);

    std::getline(rosbag, field_name, '=');
    LOG_IF(WARNING, field_name != "op")
            << "Expected op code at beginning of a record";

    int op_val = 0;
    rosbag.read(reinterpret_cast<char *>(&op_val), 1);
    LOG(INFO) << field_name << " = " << op_val;

    header_len -= (4 + field_len);
    return op_val;
}

std::tuple<long int, int, int> readBagHeader(std::ifstream &rosbag,
                                             int header_len) {
    LOG(INFO) << "\nReading Bag Header Record";

    std::string field_name;

    long int index_pos = 0;
    int conn_count = 0;
    int chunk_count = 0;

    std::tie(field_name, index_pos) = readField<long int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "index_pos")
            << "Bag Header Record Header contains unexpected field";

    std::tie(field_name, conn_count) = readField<int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "conn_count")
            << "Bag Header Record Header contains unexpected field";

    std::tie(field_name, chunk_count) = readField<int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "chunk_count")
            << "Bag Header Record Header contains unexpected field";

    LOG_IF(WARNING, header_len != 0)
            << "Parsing of header for Bag Header Record incomplete";

    // Ignore Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Bag Header Record - Data Length = " << data_len << "\n";

    rosbag.ignore(data_len);
    return std::make_tuple(index_pos, conn_count, chunk_count);
}

int readChunk(std::ifstream &rosbag, int header_len) {
    LOG(INFO) << "\nReading Chunk Record";

    auto [field_name, field_val] = readStringField(rosbag, header_len);
    LOG_IF(WARNING, field_name != "compression")
            << "Chunk Record Header contains unexpected field";

    int uncompressed_chunk_size = 0;
    std::tie(field_name, uncompressed_chunk_size) =
            readField<int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "size")
            << "Chunk Record Header contains unexpected field";

    LOG_IF(WARNING, header_len != 0)
            << "Parsing of Chunk Record Header incomplete";

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Chunk Record - Data Length = " << data_len << "\n";

    (void)readRecord(rosbag);  // Connection
    (void)readRecord(rosbag);  // Message Data

    return data_len;
}

void readConnection(std::ifstream &rosbag, int header_len) {
    LOG(INFO) << "\nReading Connection Record";

    auto [field_name, field_val] = readStringField(rosbag, header_len);
    LOG_IF(WARNING, field_name != "topic")
            << "Connection Record Header contains unexpected field";

    int conn = 0;
    std::tie(field_name, conn) = readField<int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "conn")
            << "Connection Record Header contains unexpected field";

    LOG_IF(WARNING, header_len != 0)
            << "Parsing of Connection Record Header incomplete";

    // Data - Connection Header
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Connection Record - Data Length = " << data_len << "\n";

    const std::set<std::string> expected_field_names{
            "topic",    "type",    "md5sum", "message_definition",
            "callerid", "latching"};

    while (data_len != 0) {
        std::tie(field_name, field_val) = readStringField(rosbag, data_len);
        LOG_IF(WARNING, expected_field_names.find(field_name) ==
                                expected_field_names.end())
                << "Connection Header contains unexpected field";
    }
}

void readMessageData(std::ifstream &rosbag, int header_len) {
    LOG(INFO) << "\nReading Message Data Record";

    std::string field_name;
    int conn = 0;
    std::tie(field_name, conn) = readField<int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "conn")
            << "Message Data Record Header contains unexpected field";

    long int time = 0;
    std::tie(field_name, time) = readField<long int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "time")
            << "Message Data Record Header contains unexpected field";

    LOG_IF(WARNING, header_len != 0)
            << "Parsing of Message Data Record Header incomplete";

    // Data - Serialized Message Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Message Data Record - Data Length =" << data_len;

    (void)readPointCloud2(rosbag, data_len);
}

void readIndexData(std::ifstream &rosbag, int header_len) {
    LOG(INFO) << "\nReading Index Data Record";

    std::map<std::string, int> fields{{"conn", 0}, {"count", 0}, {"ver", 0}};

    std::string field_name;
    int field_val = 0;
    for (int i = 0; i < 3; i++) {
        std::tie(field_name, field_val) =
                readField<typeof field_val>(rosbag, header_len);
        LOG_IF(WARNING, fields.find(field_name) == fields.end())
                << "Index Data Record Header contains unexpected field";
        fields[field_name] = field_val;
    }

    LOG_IF(WARNING, header_len != 0)
            << "Parsing of Index Data Record Header incomplete";

    LOG_IF(WARNING, fields["ver"] != 1)
            << "Index Data Record version " << fields["ver"] << "not supported";

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Index Data Record - Data Length = " << data_len;

    long int time = 0;
    rosbag.read(reinterpret_cast<char *>(&time), sizeof(time));
    LOG(INFO) << "time = " << time;

    int offset = 0;
    rosbag.read(reinterpret_cast<char *>(&offset), sizeof(offset));
    LOG(INFO) << "offset = " << offset;
}

void readChunkInfo(std::ifstream &rosbag, int header_len) {
    LOG(INFO) << "\nReading Chunk Info Record";

    std::string field_name;
    int version = 0;
    std::tie(field_name, version) = readField<int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "ver")
            << "Chunk Info Record Header contains unexpected field";

    long int chunk_pos = 0;
    std::tie(field_name, chunk_pos) = readField<long int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "chunk_pos")
            << "Chunk Info Record Header contains unexpected field";

    long int start_time = 0;
    std::tie(field_name, start_time) = readField<long int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "start_time")
            << "Chunk Info Record Header contains unexpected field";

    long int end_time = 0;
    std::tie(field_name, end_time) = readField<long int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "end_time")
            << "Chunk Info Record Header contains unexpected field";

    int count = 0;
    std::tie(field_name, count) = readField<int>(rosbag, header_len);
    LOG_IF(WARNING, field_name != "count")
            << "Chunk Info Record Header contains unexpected field";

    LOG_IF(WARNING, header_len != 0)
            << "Parsing of Chunk Info Record Header incomplete";

    LOG_IF(WARNING, version != 1)
            << "Chunk Info Record version " << version << "not supported";

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Chunk Info Record - Data Length = " << data_len;

    int conn = 0;
    rosbag.read(reinterpret_cast<char *>(&conn), sizeof(conn));
    LOG(INFO) << "conn = " << conn;

    rosbag.read(reinterpret_cast<char *>(&count), sizeof(count));
    LOG(INFO) << "count = " << count;
}