#include "bag_utils.hpp"

#include <glog/logging.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "sensor_msgs.hpp"

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
    DLOG(INFO) << field_name << " = " << field_val;
    header_len -= (field_len + 4);

    return std::make_tuple(field_name, field_val);
}

std::map<std::string, std::shared_ptr<char[]>> readRecordHeader(
        std::ifstream &rosbag, int &header_len) {
    int field_len = 0;
    std::string field_name;
    std::map<std::string, std::shared_ptr<char[]>> fields;

    while (header_len != 0) {
        rosbag.read(reinterpret_cast<char *>(&field_len), 4);
        std::getline(rosbag, field_name, '=');
        const int field_val_nbytes =
                field_len - 1 - static_cast<int>(field_name.size());

        std::shared_ptr<char[]> buffer(new char[field_val_nbytes]);
        rosbag.read(buffer.get(), field_val_nbytes);
        fields.insert({{field_name, std::move(buffer)}});
        header_len -= (4 + field_len);
    }

    return fields;
}

void readRecord(std::ifstream &rosbag,
                int &chunk_count,
                std::vector<std::tuple<std::string, std::string>> &connections,
                std::string &pcl_save_path) {
    // Read Header
    int header_len = 0;
    rosbag.read(reinterpret_cast<char *>(&header_len), 4);
    DLOG(INFO) << "Record Header Length = " << header_len << std::endl;
    auto fields = readRecordHeader(rosbag, header_len);

    int opval = 0;
    if (fields.find("op") != fields.end()) {
        opval = static_cast<int>(
                *reinterpret_cast<uint8_t *>(fields["op"].get()));
        DLOG(INFO) << "opval = " << opval;
    }

    // Read Header

    switch (opval) {
        case 3:
            LOG(FATAL) << "Only one bag header expected which is supposed to "
                          "be read at the beginning of the main function";
            break;

        case 5:
            readChunk(rosbag, fields, chunk_count);
            break;

        case 7:
            readConnection(rosbag, fields, connections);
            break;

        case 2:
            readMessageData(rosbag, fields, connections, pcl_save_path);
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
}

std::tuple<long int, int, int> readBagHeader(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> fields) {
    DLOG(WARNING) << "Reading Bag Header Record";

    LOG_IF(WARNING, fields.find("index_pos") == fields.end())
            << "Bag Header Record Header contains unexpected field";
    auto index_pos = *reinterpret_cast<long int *>(fields["index_pos"].get());
    DLOG(INFO) << "index_pos = " << index_pos;

    LOG_IF(WARNING, fields.find("conn_count") == fields.end())
            << "Bag Header Record Header contains unexpected field";
    auto conn_count = *reinterpret_cast<int *>(fields["conn_count"].get());
    DLOG(INFO) << "conn_count = " << conn_count;

    LOG_IF(WARNING, fields.find("chunk_count") == fields.end())
            << "Bag Header Record Header contains unexpected field";
    auto chunk_count = *reinterpret_cast<int *>(fields["chunk_count"].get());
    DLOG(INFO) << "chunk_count = " << chunk_count;

    // Ignore Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    DLOG(INFO) << "Bag Header Record - Data Length = " << data_len << "\n";

    rosbag.ignore(data_len);
    return std::make_tuple(index_pos, conn_count, chunk_count);
}

void readChunk(std::ifstream &rosbag,
               std::map<std::string, std::shared_ptr<char[]>> fields,
               int &chunk_count) {
    DLOG(WARNING) << "Reading Chunk Record";
    chunk_count++;

    LOG_IF(WARNING, fields.find("compression") == fields.end())
            << "Chunk Record Header contains unexpected field";
    auto compression = std::string(fields["compression"].get());
    DLOG(INFO) << "compression = "
               << compression.substr(0, compression.size() - 2);

    LOG_IF(WARNING, fields.find("size") == fields.end())
            << "Chunk Record Header contains unexpected field";
    auto uncompressed_chunk_size =
            *reinterpret_cast<int *>(fields["size"].get());
    DLOG(INFO) << "uncompressed_chunk_size = " << uncompressed_chunk_size;

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    DLOG(INFO) << "Chunk Record - Data Length = " << data_len << "\n";
}

void readConnection(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> fields,
        std::vector<std::tuple<std::string, std::string>> &connections) {
    DLOG(WARNING) << "Reading Connection Record";

    LOG_IF(WARNING, fields.find("topic") == fields.end())
            << "Connection Record Header contains unexpected field";
    auto topic = std::string(fields["topic"].get());
    DLOG(INFO) << "topic = " << topic.substr(0, topic.size());

    LOG_IF(WARNING, fields.find("conn") == fields.end())
            << "Connection Record Header contains unexpected field";
    auto conn = *reinterpret_cast<int *>(fields["conn"].get());
    DLOG(INFO) << "conn = " << conn;

    // Data - Connection Header
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    DLOG(INFO) << "Connection Record - Data Length = " << data_len << "\n";

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
    connections.emplace_back(
            std::move(std::make_tuple(data_fields["type"], topic)));
}

void readMessageData(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> fields,
        std::vector<std::tuple<std::string, std::string>> &connections,
        std::string &pcl_save_path) {
    DLOG(WARNING) << "Reading Message Data Record";

    LOG_IF(WARNING, fields.find("conn") == fields.end())
            << "Message Data Record Header contains unexpected field";
    auto conn = *reinterpret_cast<int *>(fields["conn"].get());
    DLOG(INFO) << "conn = " << conn;
    LOG_IF(WARNING, conn >= connections.size()) << "Unexpected Connection";

    LOG_IF(WARNING, fields.find("time") == fields.end())
            << "Message Data Record Header contains unexpected field";
    auto time = *reinterpret_cast<long int *>(fields["time"].get());
    DLOG(INFO) << "time = " << time;

    // Data - Serialized Message Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    DLOG(INFO) << "Message Data Record - Data Length =" << data_len;

    auto [msg_type, topic] = connections[conn];

    if (msg_type == "sensor_msgs/PointCloud2") {
        parsePointCloud2Msg(rosbag, data_len, topic, pcl_save_path);
        return;
    }
    LOG(WARNING) << "MSG TYPE" << msg_type.c_str();
    rosbag.ignore(data_len);
}

void readIndexData(std::ifstream &rosbag,
                   std::map<std::string, std::shared_ptr<char[]>> fields) {
    DLOG(WARNING) << "Reading Index Data Record";

    LOG_IF(WARNING, fields.find("conn") == fields.end())
            << "Index Data Record Header contains unexpected field";
    auto conn = *reinterpret_cast<int *>(fields["conn"].get());
    DLOG(INFO) << "conn = " << conn;

    LOG_IF(WARNING, fields.find("count") == fields.end())
            << "Index Data Record Header contains unexpected field";
    auto count = *reinterpret_cast<int *>(fields["count"].get());
    DLOG(INFO) << "count = " << count;

    LOG_IF(WARNING, fields.find("ver") == fields.end())
            << "Index Data Record Header contains unexpected field";
    auto ver = *reinterpret_cast<int *>(fields["ver"].get());
    DLOG(INFO) << "ver = " << ver;

    LOG_IF(WARNING, ver != 1)
            << "Index Data Record version " << ver << "not supported";

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    DLOG(WARNING) << "Index Data Record - Data Length = " << data_len;
    for (int i = 0; i < count; i++) {
        long int time = 0;
        rosbag.read(reinterpret_cast<char *>(&time), sizeof(time));
        DLOG(INFO) << "time = " << time;

        int offset = 0;
        rosbag.read(reinterpret_cast<char *>(&offset), sizeof(offset));
        DLOG(INFO) << "offset = " << offset;
    }
}

void readChunkInfo(std::ifstream &rosbag,
                   std::map<std::string, std::shared_ptr<char[]>> fields) {
    DLOG(WARNING) << "Reading Chunk Info Record";

    LOG_IF(WARNING, fields.find("ver") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto ver = *reinterpret_cast<int *>(fields["ver"].get());
    DLOG(INFO) << "ver = " << ver;
    LOG_IF(WARNING, ver != 1)
            << "Chunk Info Record version " << ver << "not supported";

    LOG_IF(WARNING, fields.find("chunk_pos") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto chunk_pos = *reinterpret_cast<long int *>(fields["chunk_pos"].get());
    DLOG(INFO) << "chunk_pos = " << chunk_pos;

    LOG_IF(WARNING, fields.find("start_time") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto start_time = *reinterpret_cast<long int *>(fields["start_time"].get());
    DLOG(INFO) << "start_time = " << start_time;

    LOG_IF(WARNING, fields.find("end_time") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto end_time = *reinterpret_cast<long int *>(fields["end_time"].get());
    DLOG(INFO) << "end_time = " << end_time;

    LOG_IF(WARNING, fields.find("count") == fields.end())
            << "Chunk Info Record Header contains unexpected field";
    auto conn_count = *reinterpret_cast<int *>(fields["count"].get());
    DLOG(INFO) << "count = " << conn_count;

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    DLOG(INFO) << "Chunk Info Record - Data Length = " << data_len;

    for (int i = 0; i < conn_count; i++) {
        int conn = 0;
        rosbag.read(reinterpret_cast<char *>(&conn), sizeof(conn));
        DLOG(INFO) << "conn = " << conn;

        int msg_count = 0;
        rosbag.read(reinterpret_cast<char *>(&msg_count), sizeof(msg_count));
        DLOG(INFO) << "count = " << msg_count;
    }
}