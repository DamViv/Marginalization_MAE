#include "node.h"

Node::Node(unsigned int key_)
    : nb_visit(1),
      key(key_),
      R(Eigen::Vector3d::Zero()),
      T(Eigen::Vector3d::Zero()) {
}

Node::~Node() {
}