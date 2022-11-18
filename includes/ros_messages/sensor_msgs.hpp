#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

class PointFieldMsg {
public:
    virtual void readDataFromStream(std::ifstream& rosbag) = 0;
    virtual std::vector<double> getData() const = 0;
    virtual const std::string& getName() const = 0;
    virtual uint32_t getOffset() const = 0;
    virtual uint32_t sizeofData() const = 0;
    virtual ~PointFieldMsg() = default;
};

template <class T>
class TypedPointFieldMsg : public PointFieldMsg {
public:
    explicit TypedPointFieldMsg(const std::string& name,
                                uint32_t offset,
                                unsigned long size)
        : name_(name), offset_(offset) {
        data_vec_.reserve(size);
    }

    void readDataFromStream(std::ifstream& rosbag) override {
        rosbag.read(reinterpret_cast<char*>(&data_), sizeof(data_));
        data_vec_.emplace_back(static_cast<double>(data_));
    }

    const std::string& getName() const override { return name_; }
    uint32_t getOffset() const override { return offset_; }
    std::vector<double> getData() const override { return data_vec_; }
    uint32_t sizeofData() const override { return sizeof(T); }

private:
    std::string name_;
    uint32_t offset_{};

    T data_{};
    std::vector<double> data_vec_;
};

void insertPointField(const std::string& name,
                      uint32_t offset,
                      uint8_t datatype,
                      unsigned long size,
                      std::vector<std::shared_ptr<PointFieldMsg>>& fields_ptr);

std::vector<std::shared_ptr<PointFieldMsg>> parsePointFieldMsg(
        std::ifstream& rosbag,
        int& data_len,
        unsigned long size,
        int32_t num_point_fields);

void parsePointCloud2Msg(std::ifstream& rosbag,
                         int data_len,
                         const std::string& topic,
                         const std::string& pcl_save_path);
