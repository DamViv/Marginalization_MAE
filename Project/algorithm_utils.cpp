#include "algorithm_utils.h"
bool MI_descend(std::vector<double> a, std::vector<double> b) {
    return a[2] > b[2];
}

bool id_ascend(relation& a, relation& b) {
    if (a.id1 == b.id1) {
        return a.id2 < b.id2;
    } else {
        return a.id1 < b.id2;
    }
}