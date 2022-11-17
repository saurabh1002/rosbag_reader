#include "io_utils.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

std::vector<std::vector<double>> transpose(
        const std::vector<std::vector<double>>& vec) {
    if (vec.empty()) {
        return vec;
    }

    std::vector<std::vector<double>> trans_vec(vec[0].size(),
                                               std::vector<double>());

    for (int i = 0; i < vec.size(); i++) {
        for (int j = 0; j < vec[i].size(); j++) {
            trans_vec[j].push_back(vec[i][j]);
        }
    }

    return trans_vec;
}

void savePointCloud(const std::vector<std::vector<double>>& data,
                    const std::vector<std::string>& field_names,
                    const std::string& output_path,
                    int32_t idx) {
    const std::filesystem::path out_path(output_path);
    if (!std::filesystem::is_directory(out_path)) {
        std::filesystem::create_directories(out_path);
    }

    std::ofstream ply_out;
    const std::string filename = std::to_string(idx) + ".ply";
    ply_out.open(out_path / filename,
                 std::ios_base::out | std::ios_base::binary);

    ply_out << "ply\n"
            << "format binary_little_endian 1.0\n";
    ply_out << "comment Created by SSG\n";
    ply_out << "element vertex " << data[0].size() << "\n";
    for (const auto& name : field_names) {
        ply_out << "property double " << name << "\n";
    }
    ply_out << "end_header\n";

    auto points = transpose(data);
    for (auto& point : points) {
        for (auto val : point) {
            ply_out.write(reinterpret_cast<char*>(&val), sizeof(val));
        }
    }
}