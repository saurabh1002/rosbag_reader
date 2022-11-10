#include "utils.hpp"

#include <cassert>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
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
    std::cout << field_name << " = " << field_val << "\n";
    header_len -= (field_len + 4);

    return std::make_tuple(field_name, field_val);
}

int readRecord(std::ifstream &rosbag) {
    // Read Header Length
    int header_len = 0;
    rosbag.read(reinterpret_cast<char *>(&header_len), 4);
    std::cout << "Header Length = " << header_len << std::endl;

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
            break;
        }
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

        default:
            retval = 0;
            std::cerr << "No record reader for opval = " << opval << std::endl;
            break;
    }
    return retval;
}

int readOp(std::ifstream &rosbag, int &header_len) {
    std::string field_name;
    int field_len = 0;
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);

    std::getline(rosbag, field_name, '=');
    assert(((void)"Wrong field name for bag header record, expected op",
            field_name == "op"));

    int op_val = 0;
    rosbag.read(reinterpret_cast<char *>(&op_val), 1);
    std::cout << field_name.c_str() << " = " << op_val << "\n";

    header_len -= (4 + field_len);
    return op_val;
}

std::tuple<long int, int, int> readBagHeader(std::ifstream &rosbag,
                                             int header_len) {
    std::cout << "Reading Bag Header ........." << std::endl;

    std::string field_name;

    const std::set<std::string> expected_field_names{"index_pos", "conn_count",
                                                     "chunk_count"};

    long int index_pos = 0;
    int conn_count = 0;
    int chunk_count = 0;

    std::tie(field_name, index_pos) = readField<long int>(rosbag, header_len);
    assert(((void)"Connection Header contains wrong field",
            expected_field_names.find(field_name) !=
                    expected_field_names.end()));

    std::tie(field_name, conn_count) = readField<int>(rosbag, header_len);
    assert(((void)"Connection Header contains wrong field",
            expected_field_names.find(field_name) !=
                    expected_field_names.end()));

    std::tie(field_name, chunk_count) = readField<int>(rosbag, header_len);
    assert(((void)"Connection Header contains wrong field",
            expected_field_names.find(field_name) !=
                    expected_field_names.end()));

    assert(((void)"Header not read completely", header_len == 0));

    // Ignore Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    std::cout << "Data Length = " << data_len << "\n\n";
    rosbag.ignore(data_len);
    return std::make_tuple(index_pos, conn_count, chunk_count);
}

int readChunk(std::ifstream &rosbag, int header_len) {
    std::cout << "Reading Chunk ........." << std::endl;

    const std::set<std::string> expected_field_names{"compression", "size"};

    auto [field_name, field_val] = readStringField(rosbag, header_len);
    assert(((void)"Connection Header contains wrong field",
            expected_field_names.find(field_name) !=
                    expected_field_names.end()));

    int uncompressed_chunk_size = 0;
    std::tie(field_name, uncompressed_chunk_size) =
            readField<int>(rosbag, header_len);
    assert(((void)"Connection Header contains wrong field",
            expected_field_names.find(field_name) !=
                    expected_field_names.end()));

    assert(((void)"Header not read completely", header_len == 0));

    // Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    std::cout << "Compressed Chunk Size = " << data_len << "\n\n";

    (void)readRecord(rosbag);  // Connection
    (void)readRecord(rosbag);  // Message Data

    return data_len;
}

void readConnection(std::ifstream &rosbag, int header_len) {
    std::cout << "Reading Connection ........." << std::endl;
    auto [field_name, field_val] = readStringField(rosbag, header_len);
    assert(((void)"Wrong field name for Connection record, expected topic",
            field_name == "topic"));

    int conn = 0;
    std::tie(field_name, conn) = readField<int>(rosbag, header_len);
    assert(((void)"Wrong field name for Connection record, expected conn",
            field_name == "conn"));

    assert(((void)"Header not read completely", header_len == 0));

    // Data - Connection Header
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);

    const std::set<std::string> expected_field_names{
            "topic",    "type",    "md5sum", "message_definition",
            "callerid", "latching"};

    while (data_len != 0) {
        std::tie(field_name, field_val) = readStringField(rosbag, data_len);
        assert(((void)"Connection Header contains wrong field",
                expected_field_names.find(field_name) !=
                        expected_field_names.end()));
    }
}

void readMessageData(std::ifstream &rosbag, int header_len) {
    std::cout << "Reading Message Data ........." << std::endl;

    std::set<std::string> expected_field_names{"conn", "time"};

    std::string field_name;
    int conn = 0;
    std::tie(field_name, conn) = readField<int>(rosbag, header_len);
    assert(((void)"Connection Header contains wrong field",
            expected_field_names.find(field_name) !=
                    expected_field_names.end()));

    long int time = 0;
    std::tie(field_name, time) = readField<long int>(rosbag, header_len);
    assert(((void)"Connection Header contains wrong field",
            expected_field_names.find(field_name) !=
                    expected_field_names.end()));

    assert(((void)"Header not read completely", header_len == 0));

    // Data - Serialized Message Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    std::printf("Message data - Data Len = %d\n", data_len);
    (void)readPointCloud2(rosbag, data_len);
}