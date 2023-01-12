#include "rosbag.hpp"

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "parse.hpp"
#include "sensor_msgs.hpp"

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
                const std::string &pcl_save_path) {
    // Read Header
    int header_len = 0;
    rosbag.read(reinterpret_cast<char *>(&header_len), 4);
    auto fields = readRecordHeader(rosbag, header_len);

    int opval = 0;
    if (fields.find("op") != fields.end()) {
        opval = int{*reinterpret_cast<uint8_t *>(fields["op"].get())};
    }

    // Read Header

    switch (opval) {
        case 3:
            "be read at the beginning of the main function";
            break;

        case 5:
            readChunkRecord(rosbag, fields, chunk_count);
            break;

        case 7:
            readConnectionRecord(rosbag, fields, connections);
            break;

        case 2:
            readMessageDataRecord(rosbag, fields, connections, pcl_save_path);
            break;

        case 4:
            readIndexDataRecord(rosbag, fields);
            break;

        case 6:
            readChunkInfoRecord(rosbag, fields);
            break;

        default:
            break;
    }
}

std::tuple<long int, int, int> readBagHeaderRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields) {
    auto index_pos = *reinterpret_cast<long int *>(fields["index_pos"].get());
    auto conn_count = *reinterpret_cast<int *>(fields["conn_count"].get());
    auto chunk_count = *reinterpret_cast<int *>(fields["chunk_count"].get());

    // Ignore Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);

    rosbag.ignore(data_len);
    return std::make_tuple(index_pos, conn_count, chunk_count);
}

void readChunkRecord(std::ifstream &rosbag,
                     std::map<std::string, std::shared_ptr<char[]>> &fields,
                     int &chunk_count) {
    chunk_count++;

    auto compression = std::string(fields["compression"].get());
    auto uncompressed_chunk_size =
            *reinterpret_cast<int *>(fields["size"].get());

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
}

void readConnectionRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields,
        std::vector<std::tuple<std::string, std::string>> &connections) {
    auto topic = std::string(fields["topic"].get());
    auto conn = *reinterpret_cast<int *>(fields["conn"].get());

    // Data - Connection Header
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);

    std::map<std::string, std::string> data_fields{
            {"topic", ""},    {"type", ""},
            {"md5sum", ""},   {"message_definition", ""},
            {"callerid", ""}, {"latching", ""}};

    std::string field_name;
    std::string field_val;
    while (data_len != 0) {
        std::tie(field_name, field_val) = readStringField(rosbag, data_len);
        data_fields[field_name] = field_val;
    }
    connections.emplace_back(
            std::move(std::make_tuple(data_fields["type"], topic)));
}

void readMessageDataRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields,
        const std::vector<std::tuple<std::string, std::string>> &connections,
        const std::string &pcl_save_path) {
    auto conn = *reinterpret_cast<int *>(fields["conn"].get());
    auto time = *reinterpret_cast<long int *>(fields["time"].get());

    // Data - Serialized Message Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);

    auto [msg_type, topic] = connections[conn];

    if (msg_type == "sensor_msgs/PointCloud2") {
        parsePointCloud2Msg(rosbag, data_len, topic, pcl_save_path);
        return;
    }
    rosbag.ignore(data_len);
}

void readIndexDataRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields) {
    auto conn = *reinterpret_cast<int *>(fields["conn"].get());
    auto count = *reinterpret_cast<int *>(fields["count"].get());
    auto ver = *reinterpret_cast<int *>(fields["ver"].get());

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    for (int i = 0; i < count; i++) {
        long int time = 0;
        rosbag.read(reinterpret_cast<char *>(&time), sizeof(time));

        int offset = 0;
        rosbag.read(reinterpret_cast<char *>(&offset), sizeof(offset));
    }
}

void readChunkInfoRecord(
        std::ifstream &rosbag,
        std::map<std::string, std::shared_ptr<char[]>> &fields) {
    auto ver = *reinterpret_cast<int *>(fields["ver"].get());
    auto chunk_pos = *reinterpret_cast<long int *>(fields["chunk_pos"].get());
    auto start_time = *reinterpret_cast<long int *>(fields["start_time"].get());
    auto end_time = *reinterpret_cast<long int *>(fields["end_time"].get());
    auto conn_count = *reinterpret_cast<int *>(fields["count"].get());

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);

    for (int i = 0; i < conn_count; i++) {
        int conn = 0;
        rosbag.read(reinterpret_cast<char *>(&conn), sizeof(conn));

        int msg_count = 0;
        rosbag.read(reinterpret_cast<char *>(&msg_count), sizeof(msg_count));
    }
}
