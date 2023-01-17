#include "algorithm_utils.h"
bool MI_descend(std::pair<float, std::pair<int, int>> a, std::pair<float, std::pair<int, int>> b) {
    return a.first > b.first;
}

bool id_ascend(relation& a, relation& b) {
    if (std::stod(a.id1) == std::stod(b.id1)) {
        return std::stod(a.id2) < std::stod(b.id2);
    } else {
        return std::stod(a.id1) < std::stod(b.id1);
    }
}