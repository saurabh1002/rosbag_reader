#include <string>
#include <vector>

void savePointCloud(const std::vector<std::vector<double>>& data,
                    const std::vector<std::string>& field_names,
                    const std::string& output_path,
                    int32_t idx);