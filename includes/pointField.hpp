#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace PointField {
class PointField {
public:
    virtual void readDataFromStream(std::ifstream& rosbag) = 0;
    virtual std::vector<double> getData() = 0;
    virtual const std::string& getName() = 0;
    virtual uint32_t getOffset() = 0;
    virtual uint32_t sizeofData() = 0;
    virtual ~PointField() = default;
};

template <class T>
class TypedPointField : public PointField {
public:
    TypedPointField(const std::string& name,
                    uint32_t offset,
                    unsigned long size)
        : name_(name), offset_(offset) {
        data_vec_.reserve(size);
    }

    void readDataFromStream(std::ifstream& rosbag) override {
        rosbag.read(reinterpret_cast<char*>(&data_), sizeof(data_));
        data_vec_.emplace_back(static_cast<double>(data_));
    }

    const std::string& getName() override { return name_; }
    uint32_t getOffset() override { return offset_; }
    std::vector<double> getData() override { return data_vec_; }
    uint32_t sizeofData() override { return sizeof(T); }

private:
    std::string name_;
    uint32_t offset_{};

    T data_{};
    std::vector<double> data_vec_;
};

void insertPointField(const std::string& name,
                      uint32_t offset,
                      const uint8_t datatype,
                      const unsigned long size,
                      std::vector<std::shared_ptr<PointField>>& fields_ptr);

std::vector<std::shared_ptr<PointField>> read(std::ifstream& rosbag,
                                              int& data_len,
                                              unsigned long size,
                                              int32_t num_point_fields);
}  // namespace PointField
