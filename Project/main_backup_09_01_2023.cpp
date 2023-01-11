#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include "algorithm_utils.h"
#include "file_utils.h"
#include "graph_utils.h"
#include "matrix_utils.h"
#include "node.h"
#include "relation.h"

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
    // string relation_file = "intel.relations.txt";
    string relation_file = "test_case1.txt";
    string basic_path = "./data_set/";

    relation_file = basic_path + relation_file;
    std::vector<relation> relations;
    relations.reserve(3000);

    loadGraph_from_relations(relation_file, relations);
    sort(relations.begin(), --relations.end(), id_ascend);

    // Initialize graph with prior on first pose
    NonlinearFactorGraph graph;
    Values initial;

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(3.5, 0.5, 1.5));
    // noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 0.1));

    auto odom_noise_model = noiseModel::Diagonal::Sigmas(Vector3(0.8165, 0.5, 0.05));
    // noiseModel::Diagonal::shared_ptr odom_noise_model = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 0.1));

    // Create a map to link time and key
    std::map<string, unsigned int> existing_nodes;

    // Create a map of landmarks to link time and key
    std::map<string, unsigned int> landmarks;

    // Add the first node with key 0 with a Prior factor to fix the traj
    unsigned int key = 0;
    graph.add(PriorFactor<Pose2>(key, Pose2(0, 0, 0), priorNoise));
    initial.insert(key, Pose2(0, 0, 0));
    existing_nodes.insert(pair<string, unsigned int>(relations.at(0).id1, key));

    // Perform Optimization at each step using Levenberg-Marquardt
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    int count = 0;
    KeySet cl_nodes;

    for (relation rel : relations) {
        string src_node = rel.id1;
        string dest_node = rel.id2;

        // check if source node exists in exising_nodes
        if (existing_nodes.find(src_node) != existing_nodes.end()) {
            Pose2 src_pose = initial.at<Pose2>(existing_nodes[src_node]);

            // check if destination node exists
            if (existing_nodes.find(dest_node) != existing_nodes.end()) {
                graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));
                cl_nodes.insert(existing_nodes[dest_node]);
            } else {
                existing_nodes.insert(pair<string, unsigned int>(dest_node, ++key));
                graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));
                double x = (cos(src_pose.theta()) * rel.T.x() - sin(src_pose.theta()) * rel.T.y()) + src_pose.x();
                double y = (sin(src_pose.theta()) * rel.T.x() + cos(src_pose.theta()) * rel.T.y()) + src_pose.y();
                double theta = src_pose.theta() + rel.R.z();
                initial.insert(existing_nodes[dest_node], Pose2(x, y, theta));
            }
        } else if (existing_nodes.find(dest_node) != existing_nodes.end()) {
            existing_nodes.insert(pair<string, unsigned int>(src_node, ++key));
            graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));

            Pose2 dest_pose = initial.at<Pose2>(existing_nodes[dest_node]);
            double src_theta = dest_pose.theta() - rel.R.z();
            double src_x = (-cos(src_theta) * rel.T.x() + sin(src_theta) * rel.T.y()) + dest_pose.x();
            double src_y = (-sin(src_theta) * rel.T.x() - cos(src_theta) * rel.T.y()) + dest_pose.y();

            initial.insert(existing_nodes[src_node], Pose2(src_x, src_y, src_theta));
        } else {
            existing_nodes.insert(pair<string, unsigned int>(src_node, ++key));
            existing_nodes.insert(pair<string, unsigned int>(dest_node, ++key));
            graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));

            initial.insert(existing_nodes[src_node], Pose2(0, 0, 0));
            initial.insert(existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()));
        }

        result = LevenbergMarquardtOptimizer(graph, initial).optimize();

        cout.precision(4);
        cout << "Relation: " << src_node << " - " << dest_node << endl;
        Pose2 src_pose = initial.at<Pose2>(existing_nodes[src_node]);
        cout << "initial  src_pose: " << src_pose.x() << ", " << src_pose.y() << ", " << src_pose.theta() << endl;
        Pose2 dest_pose = initial.at<Pose2>(existing_nodes[dest_node]);
        cout << "initial dest_pose: " << dest_pose.x() << ", " << dest_pose.y() << ", " << dest_pose.theta() << endl;
        Pose2 result_src_pose = result.at<Pose2>(existing_nodes[src_node]);
        cout << "result  src_pose: " << result_src_pose.x() << ", " << result_src_pose.y() << ", " << result_src_pose.theta() << endl;
        Pose2 result_dest_pose = result.at<Pose2>(existing_nodes[dest_node]);
        cout << "result dest_pose: " << result_dest_pose.x() << ", " << result_dest_pose.y() << ", " << result_dest_pose.theta() << endl;
        cout << ++count << " iteration is done" << endl
             << endl;
    }
    result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    ofstream os1("coords.txt");
    for (int i = 0; i < graph.size(); ++i) {
        auto factor = graph.at(i);
        auto keys = factor->keys();

        auto src = keys[0];
        auto dest = keys[1];

        Pose2 p1 = result.at<Pose2>(src);
        Pose2 p2 = result.at<Pose2>(dest);

        Pose2 i1 = initial.at<Pose2>(src);
        Pose2 i2 = initial.at<Pose2>(dest);

        os1 << p1.x() << " " << p1.y() << " " << p2.x() << " " << p2.y() << " " << i1.x() << " " << i1.y() << " " << i2.x() << " " << i2.y() << std::endl;
    }

    KeySet nodes_to_keep = graph.keys();
    KeySet nodes_to_remove;
    nodes_to_remove.insert(existing_nodes["40"]);

    for (auto node : nodes_to_remove) {
        nodes_to_keep.erase(node);
    }

    int n_remove = nodes_to_remove.size();
    int n_keep = nodes_to_keep.size();

    KeySet markov_blanket;
    get_markov_blanket(markov_blanket, graph, nodes_to_remove);
    int n_mb = markov_blanket.size();

    Marginals marginals(graph, result);
    KeyVector keys = graph.keyVector();
    // gtsam::JointMarginal I_joint = marginals.jointMarginalInformation(keys);
    gtsam::JointMarginal I_joint = marginals.jointMarginalInformation(keys);
    cout.precision(4);
    cout << "Full joint information matrix: \n"
         << I_joint.fullMatrix() << endl
         << endl;

    ofstream os0("full joint information matrix.txt");
    os0 << I_joint.fullMatrix() << endl;
    os0.close();

    Matrix I_markov(markov_blanket.size() * DOF_3, markov_blanket.size() * DOF_3);
    I_markov.fill(0);
    compute_information_markov(I_markov, I_joint, markov_blanket);

    cout << "Information matrix in Markov blanket: \n"
         << I_markov << endl
         << endl;

    // build the hash_maps
    unordered_map<int, int> map_index_t;
    unordered_map<int, int> map_node_t;

    {
        int i = 1;
        for (auto node : markov_blanket) {
            map_index_t[i] = node;
            map_node_t[node] = i;
            ++i;
        }
    }

    Eigen::MatrixXd I_marg;

    gtsam::KeySet nodes_to_keep_mb;
    for (auto node : markov_blanket) {
        nodes_to_keep_mb.insert(node);
    }

    for (auto node : nodes_to_remove) {
        if (nodes_to_keep_mb.find(node) != nodes_to_keep_mb.end()) {
            nodes_to_keep_mb.erase(node);
        }
    }

    compute_marginalized_informaiton_markov(I_marg, I_markov, nodes_to_keep_mb, map_index_t, map_node_t);
    cout << "Marginalized Information matrix: \n"
         << I_marg << endl
         << endl;

    ofstream os2("marginalization.txt");
    os2 << I_marg << endl;
    os2.close();

    // Compute CLT tree
    vector<vector<double>> pair_MIs;
    for (int k = nodes_to_keep_mb.size(); k >= 1; --k) {
        for (int j = 1; j <= k - 1; ++j) {
            vector<double> pair_MI;
            double MI = compute_mutual_information(I_marg, j, k);
            pair_MI.push_back(j);
            pair_MI.push_back(k);
            pair_MI.push_back(MI);
            pair_MIs.push_back(pair_MI);
        }
    }

    sort(pair_MIs.begin(), pair_MIs.end(), MI_descend);

    for (int i = 0; i < pair_MIs.size(); ++i) {
        cout << "MI between " << pair_MIs[i][0] << " and " << pair_MIs[i][1] << ": " << pair_MIs[i][2] << endl;
    }

    vector<vector<int>> edges_in_CLT;
    set<int> nodes_in_CLT;

    for (int k = 0; k < pair_MIs.size(); ++k) {
        vector<int> node_pair;
        node_pair.push_back(static_cast<int>(pair_MIs[k][0]));
        node_pair.push_back(static_cast<int>(pair_MIs[k][1]));

        bool is_first_node_exist = false;
        bool is_second_node_exist = false;

        if (nodes_in_CLT.find(pair_MIs[k][0]) != nodes_in_CLT.end()) {
            is_first_node_exist = true;
        }

        if (nodes_in_CLT.find(pair_MIs[k][1]) != nodes_in_CLT.end()) {
            is_second_node_exist = true;
        }

        if (!(is_first_node_exist == true && is_second_node_exist == true)) {
            edges_in_CLT.push_back(node_pair);
            nodes_in_CLT.insert(node_pair[0]);
            nodes_in_CLT.insert(node_pair[1]);
        }
    }

    cout << "edges_in_CLT: " << endl;
    for (int i = 0; i < edges_in_CLT.size(); ++i) {
        cout << edges_in_CLT[i][0] << ", " << edges_in_CLT[i][1] << endl;
    }

    Eigen::EigenSolver<Eigen::MatrixXd> es(I_marg);
    auto eig_mat = es.pseudoEigenvalueMatrix();
    auto eig_vec = es.pseudoEigenvectors();

    cout << "The pseudo-eigenvalues of I_marg: " << endl
         << eig_mat << endl
         << endl;

    cout << "The pseudo-matrix of eigenvectors, V, is: " << endl
         << eig_vec << endl
         << endl;

    auto D_diagonal = eig_mat.diagonal().transpose();
    auto D_diagonal_size = D_diagonal.size();
    bool non_zero_columns[D_diagonal_size] = {
        false,
    };

    int count_positive = 0;
    for (int i = 0; i < D_diagonal_size; ++i) {
        if (D_diagonal(i) > 0) {
            non_zero_columns[i] = true;
            ++count_positive;
        }
    }
    Eigen::MatrixXd D(count_positive, count_positive);
    Eigen::MatrixXd U(D_diagonal_size, count_positive);

    for (int i = 0; i < D_diagonal_size; ++i) {
        for (int j = 0; j < D_diagonal_size; ++j) {
            if (non_zero_columns[i] == true && non_zero_columns[j] == true) {
                D.block(i, j, 1, 1) = eig_mat.block(i, j, 1, 1);
            }
        }

        if (non_zero_columns[i] == true) {
            U.block(0, i, D_diagonal_size, 1) = eig_vec.block(0, i, D_diagonal_size, 1);
        }
    }

    cout << "D matrix: " << endl
         << D << endl
         << endl;

    cout << "U matrix: " << endl
         << U << endl
         << endl;

    Eigen::MatrixXd Sigma = D.inverse();
    Eigen::MatrixXd J(edges_in_CLT.size() * DOF_3, I_marg.row(0).size());
    J.fill(0);

    Eigen::MatrixXd Omega(edges_in_CLT.size() * DOF_3, edges_in_CLT.size() * DOF_3);
    Omega.fill(0);

    Eigen::MatrixXd z(DOF_3, edges_in_CLT.size());
    z.fill(0);

    // Another method
    /*     Eigen::MatrixXd Sigma = I_marg.inverse();
        // int Sigma_row_length = Sigma.row(0).size();  // As Sigma is square matrix, row length == column length.

        Eigen::MatrixXd J(Sigma);
        J.fill(0);

        Eigen::MatrixXd Omega(Sigma);
        Omega.fill(0);

        Eigen::MatrixXd z(DOF_3, edges_in_CLT.size());
        z.fill(0); */

    for (int k = 0; k < edges_in_CLT.size(); ++k) {
        int i = edges_in_CLT[k][0];
        int j = edges_in_CLT[k][1];

        int i_id = 0;
        int j_id = 0;

        int idx = 1;
        for (auto node_id : nodes_to_keep_mb) {
            if (idx == i) {
                i_id = node_id;
            }
            if (idx == j) {
                j_id = node_id;
            }
            ++idx;
        }

        gtsam::Pose2 meas_i = result.at<Pose2>(i_id);
        cout << "i node number: " << i_id << endl;

        Eigen::VectorXd ti(2);
        ti << meas_i.x(), meas_i.y();
        cout << "ti: " << endl
             << ti << endl
             << endl;

        Eigen::Matrix2d Ri;
        Ri << cos(meas_i.theta()), -sin(meas_i.theta()), sin(meas_i.theta()), cos(meas_i.theta());
        cout << "Ri: " << endl
             << Ri << endl
             << endl;

        gtsam::Pose2 meas_j = result.at<Pose2>(j_id);
        cout << "j node number: " << j_id << endl;

        Eigen::VectorXd tj(2);
        tj << meas_j.x(), meas_j.y();
        cout << "tj: " << endl
             << tj << endl
             << endl;

        Eigen::Matrix2d Rj;
        Rj << cos(meas_j.theta()), -sin(meas_j.theta()), sin(meas_j.theta()), cos(meas_j.theta());
        cout << "Rj: " << endl
             << Rj << endl
             << endl;

        Eigen::Matrix2d R_perp;  // Perpendiucular rotation matrix
        R_perp << 0, 1, -1, 0;
        cout << "Perpendicular rotaion matrix: " << endl
             << R_perp << endl
             << endl;

        Eigen::MatrixXd delta_t = Ri.transpose() * (tj - ti);
        Eigen::MatrixXd delta_R = (Ri.transpose() * Rj);
        cout << "delta_t: " << endl
             << delta_t << endl
             << endl;

        cout << "delta_R: " << endl
             << delta_R << endl
             << endl;

        auto delta_theta = acos(delta_R(0, 0));

        Eigen::Matrix<double, 3, 1> delta = {delta_t(0, 0), delta_t(1, 0), delta_theta};
        z.block(0, k, DOF_3, 1) = delta;
        cout << "z: " << endl
             << z << endl
             << endl;

        Eigen::MatrixXd J_k(DOF_3, I_marg.row(0).size());
        J_k.fill(0);

        Eigen::MatrixXd tmp_matrix(DOF_3, DOF_3);
        tmp_matrix.block(0, 0, DOF_3 - 1, DOF_3 - 1) = Rj.transpose() * Ri;
        tmp_matrix.block(0, DOF_3 - 1, DOF_3 - 1, 1) = R_perp * Rj.transpose() * (ti - tj);
        Eigen::Matrix<double, 1, 3> block_row = {0, 0, 1};
        tmp_matrix.block(DOF_3 - 1, 0, 1, DOF_3) = block_row;
        J_k.block(0, (i - 1) * DOF_3, DOF_3, DOF_3) = -tmp_matrix;
        J_k.block(0, (j - 1) * DOF_3, DOF_3, DOF_3) = Eigen::Matrix3d().Identity();
        // J_k.block(0, (j - 1) * DOF_3, DOF_3, DOF_3) = (Eigen::Vector3d(1, 1, 1)).asDiagonal();

        cout << "J_k matrix: " << endl
             << J_k << endl
             << endl;

        J.block(k * DOF_3, 0, DOF_3, J.row(0).size()) = J_k;

        cout << "J matrix: " << endl
             << J << endl
             << endl;

        // Another method
        /*         Eigen::MatrixXd Omega_k = (J_k * Sigma * J_k.transpose()).inverse();
                cout << "Omega_k matrix: " << endl
                     << Omega_k << endl
                     << endl;

                Omega.block(k * DOF_3, k * DOF_3, DOF_3, DOF_3) = Omega_k;
                cout << "Omega matrix: " << endl
                     << Omega << endl
                     << endl; */
    }

    J = J * U;

    cout << "J * U: " << endl
         << J << endl
         << endl;

    for (int k = 0; k < edges_in_CLT.size(); ++k) {
        Eigen::MatrixXd inter = J * Sigma * J.transpose();
        Eigen::MatrixXd Omega_k = inter.block(k * DOF_3, k * DOF_3, DOF_3, DOF_3).inverse();
        Omega.block(k * DOF_3, k * DOF_3, DOF_3, DOF_3) = Omega_k;
    }

    // Another method
    /*     {
            cout << "J before doing something: " << endl
                 << J << endl
                 << endl;

            cout << "Omega before something: " << endl
                 << Omega << endl
                 << endl;

            Eigen::MatrixXd J_k(DOF_3, I_marg.row(0).size());
            J_k.fill(0);
            J_k.block(0, 0, DOF_3, DOF_3) = Eigen::Matrix3d().Identity();
            // J_k.block(0, 0, DOF_3, DOF_3) = (Eigen::Vector3d(1, 1, 1)).asDiagonal();

            int last_idx_mb = I_marg.row(0).size() - DOF_3;
            Eigen::MatrixXd Omega_k = (J_k * Sigma * J_k.transpose()).inverse();
            Omega.block(last_idx_mb, last_idx_mb, DOF_3, DOF_3) = Omega_k;
            cout << "Omega after something: " << endl
                 << Omega << endl
                 << endl;

            J.block(last_idx_mb, 0, DOF_3, J.row(0).size()) = J_k;
            cout << "J after doing something: " << endl
                 << J << endl
                 << endl;
        } */

    Eigen::MatrixXd I_spars = J.transpose() * Omega * J;
    I_spars = U * I_spars * U.transpose();
    cout << "I_spars: " << endl
         << I_spars << endl
         << endl;

    ofstream os3("sparcification.txt");
    os3 << I_spars << endl;
    os3.close();

    auto prod = U.transpose() * I_spars * U * Sigma;
    auto KLD = 0.5 * (prod.trace() - log(prod.determinant()) - I_spars.row(0).size());
    cout << "KLD: " << KLD << endl;

    for (int node_to_remove : nodes_to_remove) {
        for (auto it = graph.begin(); it != graph.end(); ++it) {
            auto node_pair = (*it)->keys();
            if (node_pair.size() != 1) {
                if (node_pair[0] == node_to_remove || node_pair[1] == node_to_remove) {
                    graph.erase(it);
                    --it;
                }
            }
        }
        initial.erase(node_to_remove);
    }

    for (int k = 0; k < edges_in_CLT.size(); ++k) {
        int i = edges_in_CLT[k][0];
        int j = edges_in_CLT[k][1];

        int i_key = 0;
        int j_key = 0;

        int idx = 1;
        for (auto node_key : nodes_to_keep_mb) {
            if (idx == i) {
                i_key = node_key;
            }
            if (idx == j) {
                j_key = node_key;
            }
            ++idx;
        }

        string i_id;
        string j_id;

        for (auto node : existing_nodes) {
            if (node.second == i_key) {
                i_id = node.first;
            }
            if (node.second == j_key) {
                j_id = node.first;
            }
        }

        Pose2 pi = result.at<Pose2>(i_key);
        cout << "i_key: " << i_id << endl;
        cout << "pi_x: " << pi.x() << endl;
        cout << "pi_y: " << pi.y() << endl;
        cout << "pi_theta: " << pi.theta() << endl
             << endl;

        auto new_rel_pose = z.block(0, k, DOF_3, 1);
        double new_t_x = new_rel_pose(0, 0);
        double new_t_y = new_rel_pose(1, 0);
        double new_t_theta = new_rel_pose(2, 0);

        cout << "new transition x: " << new_t_x << endl;
        cout << "new transition y: " << new_t_y << endl;
        cout << "new transition theta: " << new_t_theta << endl
             << endl;

        double pj_x = cos(pi.theta()) * new_t_x - sin(pi.theta()) * new_t_y + pi.x();
        double pj_y = sin(pi.theta()) * new_t_x + cos(pi.theta()) * new_t_y + pi.y();
        double pj_theta = new_t_theta + pi.theta();

        cout << "j_key: " << j_id << endl;
        cout << "pj_x: " << pj_x << endl;
        cout << "pj_y: " << pj_y << endl;
        cout << "pj_theta: " << pj_theta << endl
             << endl;

        Eigen::MatrixXd cov = Omega.block(k * DOF_3, k * DOF_3, DOF_3, DOF_3).inverse();
        cout << "covariance: " << endl
             << cov << endl
             << endl;
        double x_cov = cov(0, 0);
        double y_cov = cov(1, 1);
        double theta_cov = cov(2, 2);

        cout << "x_cov: " << x_cov << endl;
        cout << "y_cov: " << y_cov << endl;
        cout << "theta_cov: " << theta_cov << endl
             << endl;

        auto sparsified_noise = noiseModel::Diagonal::Sigmas(Vector3(sqrt(x_cov), sqrt(y_cov), sqrt(theta_cov)));
        graph.emplace_shared<BetweenFactor<Pose2>>(i_key, j_key, Pose2(new_t_x, new_t_y, new_t_theta), sparsified_noise);

        initial.update(j_key, Pose2(pj_x, pj_y, pj_theta));
    }

    result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    ofstream os4("sparsified_coords.txt");
    for (int i = 1; i < graph.size(); ++i) {
        auto factor = graph.at(i);
        auto keys = factor->keys();

        auto src = keys[0];
        auto dest = keys[1];

        Pose2 p1 = result.at<Pose2>(src);
        Pose2 p2 = result.at<Pose2>(dest);

        os4 << p1.x() << " " << p1.y() << " " << p2.x() << " " << p2.y() << std::endl;
    }
    os4.close();

    return 0;
}