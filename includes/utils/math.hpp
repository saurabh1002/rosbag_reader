#pragma once

#include <vector>

template <typename T>
std::vector<std::vector<T>> transpose(const std::vector<std::vector<T>>& vec) {
    if (vec.empty()) {
        return vec;
    }

    std::vector<std::vector<T>> trans_vec(vec[0].size(),
                                          std::vector<T>(vec.size()));

    for (int i = 0; i < vec.size(); i++) {
        for (int j = 0; j < vec[i].size(); j++) {
            trans_vec[j][i] = vec[i][j];
        }
    }

    return trans_vec;
}