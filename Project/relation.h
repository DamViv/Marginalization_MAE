#pragma once

#include <eigen3/Eigen/Dense>
#include <string>

class relation {
   public:
    relation(std::string id1_, std::string id2_, Eigen::Vector3d R_, Eigen::Vector3d T_);
    virtual ~relation();

    std::string id1;
    std::string id2;

    Eigen::Vector3d R;
    Eigen::Vector3d T;
};

bool id_compare(relation& a, relation& b);