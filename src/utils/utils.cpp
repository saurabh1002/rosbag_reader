#include "utils.hpp"

#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

int readOp(std::ifstream &rosbag) {
    int field_len = 0;
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    assert(((void)"Expected Field Length of 4 for field op", field_len == 4));

    std::string field_name;
    std::getline(rosbag, field_name, '=');
    assert(((void)"Wrong field name for bag header record, expected op",
            field_name == "op"));

    int opval = 0;
    rosbag.read(reinterpret_cast<char *>(&opval), 1);
    std::printf("op = %d\n", opval);
    return opval;
}

int readBagHeader(std::ifstream &rosbag) {
    std::cout << "Reading Bag Header ........." << std::endl;

    std::string field_name;
    int field_len = 0;

    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    long int index_pos = 0;
    std::getline(rosbag, field_name, '=');
    assert(((void)"Wrong field name for bag header record, expected index_pos",
            field_name == "index_pos"));
    rosbag.read(reinterpret_cast<char *>(&index_pos), 8);
    std::printf("index_pos = %ld\n", index_pos);

    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    int conn_count = 0;
    std::getline(rosbag, field_name, '=');
    assert(((void)"Wrong field name for bag header record, expected conn_count",
            field_name == "conn_count"));
    rosbag.read(reinterpret_cast<char *>(&conn_count), 4);
    std::printf("conn_count = %d\n", conn_count);

    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    int chunk_count = 0;
    std::getline(rosbag, field_name, '=');
    assert(((void)"Wrong field name for bag header record, expected "
                  "chunk_count",
            field_name == "chunk_count"));
    rosbag.read(reinterpret_cast<char *>(&chunk_count), 4);
    std::printf("chunk_count = %d\n", chunk_count);

    // Ignore Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    std::cout << "Data Length = " << data_len << "\n\n\n";
    rosbag.ignore(data_len);
    return chunk_count;
}

int readChunk(std::ifstream &rosbag) {
    std::cout << "Reading Chunk ........." << std::endl;

    int field_len = 0;
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    std::string field_name;
    std::getline(rosbag, field_name, '=');
    assert(((void)"Wrong field name for chunk record, expected compression",
            field_name == "compression"));

    const int compression_type_len =
            field_len - static_cast<int>(field_name.size()) - 1;
    std::unique_ptr<char[]> const compression_type(
            new char[compression_type_len]);
    rosbag.read(compression_type.get(), compression_type_len);
    std::printf("Compression = %s\n",
                std::string(compression_type.get()).c_str());

    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    std::getline(rosbag, field_name, '=');
    assert(((void)"Wrong field name for chunk record, expected size",
            field_name == "size"));
    int compressed_chunk_size = 0;
    rosbag.read(reinterpret_cast<char *>(&compressed_chunk_size), 4);
    std::printf("Compressed Chunk Size = %d\n", compressed_chunk_size);

    int uncompressed_chunk_size = 0;
    rosbag.read(reinterpret_cast<char *>(&uncompressed_chunk_size), 4);
    std::cout << "Uncompressed Chunk Size = " << uncompressed_chunk_size
              << "\n\n";

    auto val = readRecord(rosbag);
    return uncompressed_chunk_size;
}

int readConnection(std::ifstream &rosbag, int header_len) {
    std::cout << "Reading Connection ........." << std::endl;
    int conn = 0;
    int field_len = 0;
    std::string field_name;
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    std::getline(rosbag, field_name, '=');
    assert(((void)"Wrong field name for Connection record, expected topic",
            field_name == "topic"));
    const int topic_len = field_len - static_cast<int>(field_name.size()) - 1;
    std::unique_ptr<char[]> const topic(new char[topic_len]);
    rosbag.read(topic.get(), topic_len);
    std::printf("Topic = %s\n", std::string(topic.get()).c_str());

    int val = 0;
    std::cout << header_len << std::endl;
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    std::getline(rosbag, field_name, '=');
    std::printf("Field Name = %s\n", field_name.c_str());
    rosbag.read(reinterpret_cast<char *>(&val),
                field_len - static_cast<int>(field_name.size()) - 1);
    std::printf("Field Value = %d\n", val);
    return val;
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
    opval = readOp(rosbag);

    switch (opval) {
        case 3:
            retval = readBagHeader(rosbag);
            break;
        case 5:
            retval = readChunk(rosbag);
            break;
        case 7:
            retval = readConnection(rosbag, header_len);
            break;
        default:
            retval = 0;
            std::cerr << "No record reader for opval = " << opval << std::endl;
            break;
    }
    return retval;
}