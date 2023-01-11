#include "relation.h"

relation::relation(std::string id1_, std::string id2_, Eigen::Vector3d R_, Eigen::Vector3d T_)
    : id1(id1_),
      id2(id2_),
      R(R_),
      T(T_) {
}

relation::~relation() {
}