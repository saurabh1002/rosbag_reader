#include <fstream>

namespace timestamp {
void read(std::ifstream &rosbag, int &data_len);
}

namespace header {
void read(std::ifstream &rosbag, int &data_len);
}