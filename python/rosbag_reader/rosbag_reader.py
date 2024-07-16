# MIT License

# Copyright (c) 2024 Saurabh Gupta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
from rosbag_reader.pybind import rosbag_reader_pybind


class RosbagReader:
    def __init__(self, path_to_bag):
        self._rosbag_reader = rosbag_reader_pybind._Rosbag(path_to_bag)

    def readData(self):
        self._rosbag_reader._read_data()
        return None

    def printInfo(self):
        self._rosbag_reader._print_info()
        return None

    def printAvailableTopics(self):
        self._rosbag_reader._print_available_topics()
        return None

    def savePointCloud2AsPLY(self, topic_name: str, output_path: str):
        self._rosbag_reader._save_pointcloud2_as_ply(topic_name, output_path)
        return None

    def saveLaserScanAsPLY(self, topic_name: str, output_path: str):
        self._rosbag_reader._save_laserscan_as_ply(topic_name, output_path)
        return None
