#pragma once

#include <string>
#include <vector>

void savePointCloudAsPLY(const std::vector<std::vector<double>>& points,
                         const std::vector<std::string>& field_names,
                         const std::string& output_path,
                         int32_t file_idx);