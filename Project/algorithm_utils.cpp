#include "algorithm_utils.h"
bool MI_descend(std::pair<float, std::pair<int, int>> a, std::pair<float, std::pair<int, int>> b) {
    return a.first > b.first;
}

bool id_ascend(relation& a, relation& b) {
    if (a.id1 == b.id1) {
        return a.id2 < b.id2;
    } else {
        return a.id1 < b.id2;
    }
}