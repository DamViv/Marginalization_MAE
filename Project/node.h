#pragma once

#include <eigen3/Eigen/Dense>

class Node {
   public:
    Node(unsigned int key_);
    virtual ~Node();

    Eigen::Vector3d R;
    Eigen::Vector3d T;

    int nb_visit;
    unsigned int key;
};
