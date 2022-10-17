#pragma once
#include <fstream>

int readOp(std::ifstream &rosbag);

int readBagHeader(std::ifstream &rosbag);

int readChunk(std::ifstream &rosbag);

int readRecord(std::ifstream &rosbag);

int readConnection(std::ifstream &rosbag, int header_len);