#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>

std::map<std::string, int> fields_map{
        {"op", 0}, {"index_pos", 1}, {"conn_count", 2}, {"chunk_count", 3}};

int readBagHeader(std::ifstream &rosbag) {
    std::cout << "Reading Bag Header ........." << std::endl;

    int header_len = 0;
    rosbag.read(reinterpret_cast<char *>(&header_len), 4);
    std::cout << "Header Length = " << header_len << std::endl;

    int field_len = 0;
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    std::printf("Field Length = %d\n", field_len);

    std::string field_name;
    std::getline(rosbag, field_name, '=');
    std::printf("Field Name = %s\n", field_name.c_str());

    int opval = 0;
    rosbag.read(reinterpret_cast<char *>(&opval), 1);
    std::printf("Field Value = %d\n", opval);

    long int index_pos = 0;
    rosbag.ignore(std::numeric_limits<std::streamsize>::max(), '=');
    rosbag.read(reinterpret_cast<char *>(&index_pos), 8);
    std::printf("index_pos = %ld\n", index_pos);

    int conn_count = 0;
    rosbag.ignore(std::numeric_limits<std::streamsize>::max(), '=');
    rosbag.read(reinterpret_cast<char *>(&conn_count), 4);
    std::printf("conn_count = %d\n", conn_count);

    int chunk_count = 0;
    rosbag.ignore(std::numeric_limits<std::streamsize>::max(), '=');
    rosbag.read(reinterpret_cast<char *>(&chunk_count), 4);
    std::printf("chunk_count = %d\n", chunk_count);

    // Ignore Data
    int data_len = 0;
    rosbag.read(reinterpret_cast<char *>(&data_len), 4);
    std::cout << "Data Length = " << data_len << "\n\n\n";
    rosbag.ignore(data_len);
    return chunk_count;
}

auto readHeader(std::ifstream &rosbag) {
    int opval = 0;
    int field_len = 0;
    int chunk_count = 0;
    std::string field_name;

    // Read Op Value
    rosbag.read(reinterpret_cast<char *>(&field_len), 4);
    std::getline(rosbag, field_name, '=');
    rosbag.read(reinterpret_cast<char *>(&opval), 1);
    std::printf("Field Length = %d\n", field_len);
    std::printf("Field Name = %s\n", field_name.c_str());
    std::printf("Field Value = %d\n\n", opval);

    switch (opval) {
        default:
            chunk_count = 0;
            std::cerr << "No record reader for opval = " << opval << std::endl;
            break;
    }
    return opval;
}

void readData(std::ifstream &rosbag, int data_len, int opval) {
    switch (opval) {
        default:
            rosbag.ignore(data_len);
            break;
    }
}

int main() {
    std::string const rosbag_path = "../pcl2.bag";
    std::ifstream rosbag(rosbag_path,
                         std::ios_base::in | std::ios_base::binary);
    if (!rosbag) {
        std::cerr << "Rosbag could not be opened, please check rosbag path"
                  << std::endl;
        return 1;
    }

    std::string version;
    std::getline(rosbag, version);
    if (version.find("V2.0") == std::string::npos) {
        std::cerr << "Rosbag must be of version 2.0" << std::endl;
        return 1;
    }

    auto chunk_count = readBagHeader(rosbag);

    int header_len = 0;
    rosbag.read(reinterpret_cast<char *>(&header_len), 4);
    auto opval = readHeader(rosbag);
}