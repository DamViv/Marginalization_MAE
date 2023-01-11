#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unordered_map>

const int DOF_3 = 3;

void computeMatInfJac(gtsam::Matrix& I_markov, gtsam::NonlinearFactorGraph& graph, gtsam::Values& result, gtsam::noiseModel::Diagonal::shared_ptr& odom_noise_model, gtsam::KeySet& markov_blanket);

void compute_information_markov(gtsam::Matrix& I_markov, gtsam::JointMarginal& I_joint, gtsam::KeySet& markov_blanket);
void compute_marginalized_informaiton_markov(Eigen::MatrixXd& I_marg, gtsam::Matrix& I_markov, gtsam::KeySet& nodes_to_keep_mb, std::unordered_map<int, int>& map_index_t, std::unordered_map<int, int>& map_node_t);

double compute_mutual_information(Eigen::MatrixXd& I, int i, int j);
Eigen::MatrixXd compute_joint_marginal(Eigen::MatrixXd I, int i, int j);