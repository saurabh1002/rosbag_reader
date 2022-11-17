### C++ parser for Rosbag Files

- ##### Parse useful data from a bag file and save it to local disk
- ##### No ROS required to be installed on your system
  
- ##### Supported RosBag Version: [V2.0](http://wiki.ros.org/Bags/Format/2.0)
- ##### Supported Message Types:
  - [**`sensor_msgs/PointCloud2`**](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | **output format**: *PLY - Polygon File Format*
- ##### Supported Bag Compression Types: 
  - None


#### Install Instructions

```bash
git clone https://github.com/saurabh1002/rosbag_reader.git
cd rosbag_reader
mkdir build && cd build
cmake .. && make -j$(nproc)

```

#### Usage

- You can provide the bag file path as well as the output paths in the [config](config.yaml) file
- Currently we only support PointCloud2 message type, which can be extracted using the following command:
```bash
cd build 
./parsePointCloud2Msgs ../config.yaml
```