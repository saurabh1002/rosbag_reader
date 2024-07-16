// MIT License

// Copyright (c) 2024 Saurabh Gupta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

#include "messages.h"

namespace std_msgs {
std::string readString(std::ifstream &rosbag, const int n_bytes) {
    std::unique_ptr<char[]> const buffer(new char[n_bytes + 1]);
    buffer.get()[n_bytes] = '\0';
    rosbag.read(buffer.get(), n_bytes);
    return buffer.get();
}

Time Time::parseTime(std::ifstream &rosbag) {
    int32_t secs = 0;
    int32_t nsecs = 0;
    rosbag.read(reinterpret_cast<char *>(&secs), sizeof(secs));
    rosbag.read(reinterpret_cast<char *>(&nsecs), sizeof(nsecs));

    return Time{secs, nsecs};
}

Header Header::parseHeader(std::ifstream &rosbag) {
    uint32_t seq = 0;
    rosbag.read(reinterpret_cast<char *>(&seq), sizeof(seq));

    auto stamp = Time::parseTime(rosbag);

    int32_t len_frame_id = 0;
    rosbag.read(reinterpret_cast<char *>(&len_frame_id), sizeof(len_frame_id));
    std::string const frame_id = readString(rosbag, len_frame_id);

    return Header{seq, stamp, frame_id};
}
}  // namespace std_msgs
