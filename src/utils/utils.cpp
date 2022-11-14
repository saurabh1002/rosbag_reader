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

std::map<std::string, std::shared_ptr<char[]>> readRecordHeader(
        std::ifstream &rosbag, int &header_len) {
    std::map<std::string, std::shared_ptr<char[]>> fields;
    std::string field_name;
    int field_len = 0;
    while (header_len != 0) {
        rosbag.read(reinterpret_cast<char *>(&field_len), 4);

        std::getline(rosbag, field_name, '=');
        const int field_val_nbytes =
                field_len - 1 - static_cast<int>(field_name.size());

        std::shared_ptr<char[]> buffer(new char[field_val_nbytes]);
        rosbag.read(buffer.get(), field_val_nbytes);
        fields.insert({{field_name, buffer}});
        header_len -= (4 + field_len);
    }

    return fields;
}

int readRecord(std::ifstream &rosbag, int &chunk_count) {
    // Read Header
    int header_len = 0;
    rosbag.read(reinterpret_cast<char *>(&header_len), 4);
    LOG(INFO) << "Record Header Length = " << header_len << std::endl;
    auto fields = readRecordHeader(rosbag, header_len);

    int opval = 0;
    if (fields.find("op") != fields.end()) {
        opval = static_cast<int>(
                *reinterpret_cast<uint8_t *>(fields["op"].get()));
        LOG(INFO) << "opval = " << opval;
    }

    // Read Header
    int retval = 0;

    switch (opval) {
        case 3: {
            auto [index_pos, conn_count, chunk_count] =
                    readBagHeader(rosbag, fields);
            retval = chunk_count;
        } break;

        case 5:
            readChunk(rosbag, fields, chunk_count);
            break;

        case 7:
            readConnection(rosbag, fields);
            break;

        case 2:
            readMessageData(rosbag, fields);
            break;

        case 4:
            readIndexData(rosbag, fields);
            break;

        case 6:
            readChunkInfo(rosbag, fields);
            break;

        default:
            LOG(WARNING) << "No record reader for opval = " << opval;
            break;
    }
    return retval;
}

std::tuple<long int, int, int> readBagHeader(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> fields) {
    LOG(WARNING) << "Reading Bag Header Record";

    LOG_IF(WARNING, fields.find("index_pos") == fields.end())
            << "Bag Header Record Header contains unexpected field";
    auto index_pos = *reinterpret_cast<long int *>(fields["index_pos"].get());
    LOG(INFO) << "index_pos = " << index_pos;

    LOG_IF(WARNING, fields.find("conn_count") == fields.end())
            << "Bag Header Record Header contains unexpected field";
    auto conn_count = *reinterpret_cast<int *>(fields["conn_count"].get());
    LOG(INFO) << "conn_count = " << conn_count;

    LOG_IF(WARNING, fields.find("chunk_count") == fields.end())
            << "Bag Header Record Header contains unexpected field";
    auto chunk_count = *reinterpret_cast<int *>(fields["chunk_count"].get());
    LOG(INFO) << "chunk_count = " << chunk_count;

    // Ignore Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Bag Header Record - Data Length = " << data_len << "\n";

    rosbag.ignore(data_len);
    return std::make_tuple(index_pos, conn_count, chunk_count);
}

void readChunk(std::ifstream &rosbag,
               std::map<std::string, std::shared_ptr<char[]>> fields,
               int &chunk_count) {
    LOG(WARNING) << "Reading Chunk Record";
    chunk_count++;

    LOG_IF(WARNING, fields.find("compression") == fields.end())
            << "Chunk Record Header contains unexpected field";
    auto compression = std::string(fields["compression"].get());
    LOG(INFO) << "compression = "
              << compression.substr(0, compression.size() - 2);

    LOG_IF(WARNING, fields.find("size") == fields.end())
            << "Chunk Record Header contains unexpected field";
    auto uncompressed_chunk_size =
            *reinterpret_cast<int *>(fields["size"].get());
    LOG(INFO) << "uncompressed_chunk_size = " << uncompressed_chunk_size;

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Chunk Record - Data Length = " << data_len << "\n";
}

void readConnection(std::ifstream &rosbag,
                    std::map<std::string, std::shared_ptr<char[]>> fields) {
    LOG(WARNING) << "Reading Connection Record";

    LOG_IF(WARNING, fields.find("topic") == fields.end())
            << "Connection Record Header contains unexpected field";
    auto topic = std::string(fields["topic"].get());
    LOG(INFO) << "topic = " << topic.substr(0, topic.size());

    LOG_IF(WARNING, fields.find("conn") == fields.end())
            << "Connection Record Header contains unexpected field";
    auto conn = *reinterpret_cast<int *>(fields["conn"].get());
    LOG(INFO) << "conn = " << conn;

    // Data - Connection Header
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Connection Record - Data Length = " << data_len << "\n";

    std::map<std::string, std::string> data_fields{
            {"topic", ""},    {"type", ""},
            {"md5sum", ""},   {"message_definition", ""},
            {"callerid", ""}, {"latching", ""}};

    std::string field_name;
    std::string field_val;
    while (data_len != 0) {
        std::tie(field_name, field_val) = readStringField(rosbag, data_len);
        LOG_IF(WARNING, data_fields.find(field_name) == data_fields.end())
                << "Connection Header contains unexpected field";
        data_fields[field_name] = field_val;
    }

    if (data_fields["type"] != "sensor_msgs/PointCloud2") {
        LOG(WARNING) << "Unrecognised Message Type";
    }
}

void readMessageData(std::ifstream &rosbag,
                     std::map<std::string, std::shared_ptr<char[]>> fields) {
    LOG(WARNING) << "Reading Message Data Record";

    LOG_IF(WARNING, fields.find("conn") == fields.end())
            << "Message Data Record Header contains unexpected field";
    auto conn = *reinterpret_cast<int *>(fields["conn"].get());
    LOG(INFO) << "conn = " << conn;

    LOG_IF(WARNING, fields.find("time") == fields.end())
            << "Message Data Record Header contains unexpected field";
    auto time = *reinterpret_cast<long int *>(fields["time"].get());
    LOG(INFO) << "time = " << time;

    // Data - Serialized Message Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    LOG(INFO) << "Message Data Record - Data Length =" << data_len;

    (void)readPointCloud2(rosbag, data_len);
}

void readIndexData(std::ifstream &rosbag,
                   std::map<std::string, std::shared_ptr<char[]>> fields) {
    LOG(WARNING) << "Reading Index Data Record";

    LOG_IF(WARNING, fields.find("conn") == fields.end())
            << "Index Data Record Header contains unexpected field";
    auto conn = *reinterpret_cast<int *>(fields["conn"].get());
    LOG(INFO) << "conn = " << conn;

    LOG_IF(WARNING, fields.find("count") == fields.end())
            << "Index Data Record Header contains unexpected field";
    auto count = *reinterpret_cast<int *>(fields["count"].get());
    LOG(INFO) << "count = " << count;

    LOG_IF(WARNING, fields.find("ver") == fields.end())
            << "Index Data Record Header contains unexpected field";
    auto ver = *reinterpret_cast<int *>(fields["ver"].get());
    LOG(INFO) << "ver = " << ver;

    LOG_IF(WARNING, ver != 1)
            << "Index Data Record version " << ver << "not supported";

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

void readChunkInfo(std::ifstream &rosbag,
                   std::map<std::string, std::shared_ptr<char[]>> fields) {
    LOG(WARNING) << "Reading Chunk Info Record";

    LOG_IF(WARNING, fields.find("ver") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto ver = *reinterpret_cast<int *>(fields["ver"].get());
    LOG(INFO) << "ver = " << ver;
    LOG_IF(WARNING, ver != 1)
            << "Chunk Info Record version " << ver << "not supported";

    LOG_IF(WARNING, fields.find("chunk_pos") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto chunk_pos = *reinterpret_cast<long int *>(fields["chunk_pos"].get());
    LOG(INFO) << "chunk_pos = " << chunk_pos;

    LOG_IF(WARNING, fields.find("start_time") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto start_time = *reinterpret_cast<long int *>(fields["start_time"].get());
    LOG(INFO) << "start_time = " << start_time;

    LOG_IF(WARNING, fields.find("end_time") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto end_time = *reinterpret_cast<long int *>(fields["end_time"].get());
    LOG(INFO) << "end_time = " << end_time;

    LOG_IF(WARNING, fields.find("count") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto count = *reinterpret_cast<int *>(fields["count"].get());
    LOG(INFO) << "count = " << count;

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