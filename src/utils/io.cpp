#include "io.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "math.hpp"

void savePointCloudAsPLY(const std::vector<std::vector<double>>& points,
                         const std::vector<std::string>& field_names,
                         const std::string& output_path,
                         int32_t file_idx) {
    const std::filesystem::path out_path(output_path);
    if (!std::filesystem::is_directory(out_path)) {
        std::filesystem::create_directories(out_path);
    }

    std::ofstream ply_out;
    const std::string filename = std::to_string(file_idx) + ".ply";
    ply_out.open(out_path / filename,
                 std::ios_base::out | std::ios_base::binary);

    ply_out << "ply\n"
            << "format binary_little_endian 1.0\n";
    ply_out << "comment Created by SSG\n";
    ply_out << "element vertex " << points[0].size() << "\n";
    for (const auto& name : field_names) {
        ply_out << "property double " << name << "\n";
    }
    ply_out << "end_header\n";

    auto points_t = transpose(points);
    for (auto& point : points_t) {
        for (auto val : point) {
            ply_out.write(reinterpret_cast<char*>(&val), sizeof(val));
        }
    }
}