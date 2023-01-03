#include "relation.h"

relation::relation(std::string id1_, std::string id2_, Eigen::Vector3d R_, Eigen::Vector3d T_)
    : id1(id1_),
      id2(id2_),
      R(R_),
      T(T_) {
}

relation::~relation() {
}

bool id_compare(relation& a, relation& b) {
    if (a.id1 == b.id1) {
        return a.id2 < b.id2;
    } else {
        return a.id1 < b.id2;
    }
}