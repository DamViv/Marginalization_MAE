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
    NonlinearFactorGraph original_graph;
    Values original_initial;

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.75, 0.25, 0.1));
    // auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(3.5, 5.5, 1.5));
    // noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 0.1));

    auto odom_noise_model = noiseModel::Diagonal::Sigmas(Vector3(0.8165, 0.5, 0.05));
    // auto odom_noise_model = noiseModel::Diagonal::Sigmas(Vector3(3.5, 5.5, 1.5));
    //  noiseModel::Diagonal::shared_ptr odom_noise_model = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 0.1));

    // Create a map to link time and key
    std::map<string, unsigned int> existing_nodes;

    // Create a map of landmarks to link time and key
    std::map<string, unsigned int> landmarks;

    // Add the first node with key 0 with a Prior factor to fix the traj
    unsigned int key = 0;
    original_graph.add(PriorFactor<Pose2>(key, Pose2(0, 0, 0), priorNoise));
    default_random_engine generator;
    normal_distribution<double> prior_error_x_distribution(0.0, 0.75);
    normal_distribution<double> prior_error_y_distribution(0.0, 0.25);
    normal_distribution<double> prior_error_theta_distribution(0.0, 0.1);
    double priror_error_x = prior_error_x_distribution(generator);
    double priror_error_y = prior_error_y_distribution(generator);
    double priror_error_theta = prior_error_theta_distribution(generator);
    original_initial.insert(key, Pose2(0 + priror_error_x, 0 + priror_error_y, 0 + priror_error_theta));
    existing_nodes.insert(pair<string, unsigned int>(relations.at(0).id1, key));

    // Perform Optimization at each step using Levenberg-Marquardt
    Values result = LevenbergMarquardtOptimizer(original_graph, original_initial).optimize();

    int count = 0;
    KeySet cl_nodes;

    for (relation rel : relations) {
        string src_node = rel.id1;
        string dest_node = rel.id2;

        // check if source node exists in exising_nodes
        if (existing_nodes.find(src_node) != existing_nodes.end()) {
            Pose2 src_pose = original_initial.at<Pose2>(existing_nodes[src_node]);

            // check if destination node exists
            if (existing_nodes.find(dest_node) != existing_nodes.end()) {
                original_graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));
                cl_nodes.insert(existing_nodes[dest_node]);
            } else {
                existing_nodes.insert(pair<string, unsigned int>(dest_node, ++key));
                original_graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));

                normal_distribution<double> error_x_distribution(0.0, 0.8165);
                normal_distribution<double> error_y_distribution(0.0, 0.5);
                normal_distribution<double> error_theta_distribution(0.0, 0.05);
                double error_x = error_x_distribution(generator);
                double error_y = error_y_distribution(generator);
                double error_theta = error_theta_distribution(generator);

                double x = (cos(src_pose.theta()) * (rel.T.x() + error_x) - sin(src_pose.theta()) * (rel.T.y() + error_y)) + src_pose.x();
                double y = (sin(src_pose.theta()) * (rel.T.x() + error_x) + cos(src_pose.theta()) * (rel.T.y() + error_y)) + src_pose.y();
                double theta = src_pose.theta() + rel.R.z() + error_theta;
                original_initial.insert(existing_nodes[dest_node], Pose2(x, y, theta));
            }
        } else if (existing_nodes.find(dest_node) != existing_nodes.end()) {
            existing_nodes.insert(pair<string, unsigned int>(src_node, ++key));
            original_graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));

            Pose2 dest_pose = original_initial.at<Pose2>(existing_nodes[dest_node]);

            normal_distribution<double> error_x_distribution(0.0, 0.8165);
            normal_distribution<double> error_y_distribution(0.0, 0.5);
            normal_distribution<double> error_theta_distribution(0.0, 0.05);
            double error_x = error_x_distribution(generator);
            double error_y = error_y_distribution(generator);
            double error_theta = error_theta_distribution(generator);

            double src_theta = dest_pose.theta() - rel.R.z() + error_theta;
            double src_x = (-cos(src_theta) * (rel.T.x() + error_x) + sin(src_theta) * (rel.T.y() + error_y)) + dest_pose.x();
            double src_y = (-sin(src_theta) * (rel.T.x() + error_x) - cos(src_theta) * (rel.T.y() + error_y)) + dest_pose.y();

            original_initial.insert(existing_nodes[src_node], Pose2(src_x, src_y, src_theta));
        } else {
            existing_nodes.insert(pair<string, unsigned int>(src_node, ++key));
            existing_nodes.insert(pair<string, unsigned int>(dest_node, ++key));
            original_graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));

            original_initial.insert(existing_nodes[src_node], Pose2(0, 0, 0));

            normal_distribution<double> error_x_distribution(0.0, 0.8165);
            normal_distribution<double> error_y_distribution(0.0, 0.5);
            normal_distribution<double> error_theta_distribution(0.0, 0.05);
            double error_x = error_x_distribution(generator);
            double error_y = error_y_distribution(generator);
            double error_theta = error_theta_distribution(generator);

            original_initial.insert(existing_nodes[dest_node], Pose2(rel.T.x() + error_x, rel.T.y() + error_y, rel.R.z() + error_theta));
        }
        result = LevenbergMarquardtOptimizer(original_graph, original_initial).optimize();

        cout.precision(4);
        cout << "Relation: " << src_node << " - " << dest_node << endl;
        Pose2 src_pose = original_initial.at<Pose2>(existing_nodes[src_node]);
        cout << "initial  src_pose: " << src_pose.x() << ", " << src_pose.y() << ", " << src_pose.theta() << endl;
        Pose2 dest_pose = original_initial.at<Pose2>(existing_nodes[dest_node]);
        cout << "initial dest_pose: " << dest_pose.x() << ", " << dest_pose.y() << ", " << dest_pose.theta() << endl;

        Pose2 result_src_pose = result.at<Pose2>(existing_nodes[src_node]);
        cout << "result  src_pose: " << result_src_pose.x() << ", " << result_src_pose.y() << ", " << result_src_pose.theta() << endl;
        Pose2 result_dest_pose = result.at<Pose2>(existing_nodes[dest_node]);
        cout << "result dest_pose: " << result_dest_pose.x() << ", " << result_dest_pose.y() << ", " << result_dest_pose.theta() << endl;

        cout << ++count << " iteration is done" << endl
             << endl;
    }
    result = LevenbergMarquardtOptimizer(original_graph, original_initial).optimize();
    ofstream os1("coords_including_noise.txt");
    for (int i = 0; i < original_graph.size(); ++i) {
        auto factor = original_graph.at(i);
        auto keys = factor->keys();

        auto src = keys[0];
        auto dest = keys[1];

        Pose2 p1 = result.at<Pose2>(src);
        Pose2 p2 = result.at<Pose2>(dest);

        Pose2 i1 = original_initial.at<Pose2>(src);
        Pose2 i2 = original_initial.at<Pose2>(dest);

        os1 << p1.x() << " " << p1.y() << " " << p2.x() << " " << p2.y() << " " << i1.x() << " " << i1.y() << " " << i2.x() << " " << i2.y() << std::endl;
    }

    cout.precision(4);
    KeyVector key_vec = original_graph.keyVector();
    KeySet original_graph_keys;
    for (auto i : key_vec) {
        original_graph_keys.insert(i);
    }

    Matrix I_full_Jac(original_graph_keys.size() * DOF_3, original_graph_keys.size() * DOF_3);
    I_full_Jac.fill(0);
    computeMatInfJac(I_full_Jac, original_graph, result, odom_noise_model, original_graph_keys);

    cout.precision(4);
    cout << "Full Jacobian information matrix: \n"
         << I_full_Jac << endl
         << endl;

    /*     Eigen::MatrixXd I_full_marg;
        compute_marginalized_informaiton_markov(I_full_marg, I_full_Jac, original_graph_keys, map_full_index_t, map_full_node_t); */
    for (int k = original_graph_keys.size(); k >= 1; --k) {
        for (int j = 1; j <= k - 1; ++j) {
            Eigen::MatrixXd I_ii = I_full_Jac.block((k - 1) * DOF_3, (k - 1) * DOF_3, DOF_3, DOF_3);
            /*             cout << "I_ii: " << endl
                             << I_ii << endl
                             << endl; */

            Eigen::MatrixXd I_ij = I_full_Jac.block((k - 1) * DOF_3, (j - 1) * DOF_3, DOF_3, DOF_3);
            /*             cout << "I_ij: " << endl
                             << I_ij << endl
                             << endl; */

            Eigen::MatrixXd I_jj = I_full_Jac.block((j - 1) * DOF_3, (j - 1) * DOF_3, DOF_3, DOF_3);
            /*             cout << "I_jj: " << endl
                             << I_jj << endl
                             << endl; */

            double numerator = (I_ii + Eigen::Matrix3d::Identity()).determinant();
            double denominator = (I_ii - (I_ij * (I_jj.completeOrthogonalDecomposition()).pseudoInverse() * I_ij.transpose()) + Eigen::Matrix3d::Identity()).determinant();

            double MI = 0.5 * log(numerator / denominator);
            cout << "MI(" << k - 1 << "-" << j - 1 << "): " << MI << endl;
        }
    }
    /*     auto test = original_initial.at<Pose2>(0);
        cout << test.x() << ", " << test.y() << endl; */

    KeySet nodes_to_keep = original_graph.keys();
    KeySet nodes_to_remove;
    // nodes_to_remove.insert(existing_nodes["30"]);
    nodes_to_remove.insert(existing_nodes["40"]);

    for (auto node : nodes_to_remove) {
        nodes_to_keep.erase(node);
    }

    int n_remove = nodes_to_remove.size();
    int n_keep = nodes_to_keep.size();

    KeySet markov_blanket;
    get_markov_blanket(markov_blanket, original_graph, nodes_to_remove);
    cout << "Nodes in markov blanket: " << endl;
    for (auto node : markov_blanket) {
        cout << node << endl;
    }
    cout << endl;
    int n_mb = markov_blanket.size();

    Matrix I_markov(markov_blanket.size() * DOF_3, markov_blanket.size() * DOF_3);
    I_markov.fill(0);
    computeMatInfJac(I_markov, original_graph, result, odom_noise_model, markov_blanket);

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
    Graph g(nodes_to_keep_mb.size() + 1);
    // vector<vector<double>> pair_MIs;
    for (int k = nodes_to_keep_mb.size(); k >= 1; --k) {
        for (int j = 1; j <= k - 1; ++j) {
            // vector<double> pair_MI;
            double MI = compute_mutual_information(I_marg, j, k);
            g.AddWeightedEdge(j, k, MI);
        }
    }

    g.kruskal();
    g.print();

    /*     for (int i = 0; i < pair_MIs.size(); ++i) {
            cout << "MI between " << pair_MIs[i][0] << " and " << pair_MIs[i][1] << ": " << pair_MIs[i][2] << endl;
        } */

    vector<vector<int>> edges_in_CLT;
    // set<int> nodes_in_CLT;

    for (int k = 0; k < g.size(); ++k) {
        pair<int, int> one_edge = g.get_edge(k);
        vector<int> node_pair;
        node_pair.push_back(one_edge.first);
        node_pair.push_back(one_edge.second);
        edges_in_CLT.push_back(node_pair);
    }

    cout << "edges_in_CLT: " << endl;
    for (int i = 0; i < edges_in_CLT.size(); ++i) {
        cout << edges_in_CLT[i][0] << ", " << edges_in_CLT[i][1] << endl;
    }

    Eigen::EigenSolver<Eigen::MatrixXd> es(I_marg);
    Eigen::MatrixXd eig_mat = es.pseudoEigenvalueMatrix();
    Eigen::MatrixXd eig_vec = es.pseudoEigenvectors();

    cout << "The pseudo-eigenvalues of I_marg: " << endl
         << eig_mat << endl
         << endl;

    cout << "The pseudo-matrix of eigenvectors, V, is: " << endl
         << eig_vec << endl
         << endl;

    Eigen::MatrixXd D_diagonal = eig_mat.diagonal().transpose();
    int D_diagonal_size = D_diagonal.size();
    bool positives[D_diagonal_size] = {
        false,
    };

    int count_positive = 0;
    for (int i = 0; i < D_diagonal_size; ++i) {
        if (D_diagonal(i) > 0.00001) {
            positives[i] = true;
            ++count_positive;
        }
    }
    Eigen::MatrixXd D(count_positive, count_positive);
    D.fill(0);

    Eigen::MatrixXd U(D_diagonal_size, count_positive);
    U.fill(0);

    // There is some errors in for loop
    int offset_negative = 0;
    for (int i = 0; i < D_diagonal_size; ++i) {
        if (positives[i] == true) {
            D(i - offset_negative, i - offset_negative) = eig_mat(i, i);

            cout << "D(constructing): " << endl
                 << D << endl
                 << endl;

            U.block(0, i - offset_negative, D_diagonal_size, 1) = eig_vec.block(0, i, D_diagonal_size, 1);
            cout << "U(constructing): " << endl
                 << U << endl
                 << endl;
        } else {
            ++offset_negative;
        }
    }

    cout << "D matrix: " << endl
         << D << endl
         << endl;

    cout << "U matrix: " << endl
         << U << endl
         << endl;

    Eigen::MatrixXd Sigma = D.inverse();
    // Eigen::MatrixXd Sigma = I_marg.inverse();

    Eigen::MatrixXd J(edges_in_CLT.size() * DOF_3, I_marg.row(0).size());
    J.fill(0);

    Eigen::MatrixXd Omega_breve(edges_in_CLT.size() * DOF_3, edges_in_CLT.size() * DOF_3);
    Omega_breve.fill(0);

    Eigen::MatrixXd z(DOF_3, edges_in_CLT.size());
    z.fill(0);

    Eigen::MatrixXd Phi(edges_in_CLT.size() * DOF_3, edges_in_CLT.size() * DOF_3);
    Phi.fill(0);

    Eigen::MatrixXd Upsilon(edges_in_CLT.size() * DOF_3, edges_in_CLT.size() * DOF_3);
    Upsilon.fill(0);

    // Another method
    /*     Eigen::MatrixXd Sigma = I_marg.inverse();
        // int Sigma_row_length = Sigma.row(0).size();  // As Sigma is square matrix, row length == column length.

        Eigen::MatrixXd J(Sigma);
        J.fill(0);

        Eigen::MatrixXd Omega_breve(Sigma);
        Omega_breve.fill(0);

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
        /*         Eigen::MatrixXd Omega_breve_k = (J_k * Sigma * J_k.transpose()).inverse();
                cout << "Omega_breve_k matrix: " << endl
                     << Omega_breve_k << endl
                     << endl;

                Omega_breve.block(k * DOF_3, k * DOF_3, DOF_3, DOF_3) = Omega_breve_k;
                cout << "Omega_breve matrix: " << endl
                     << Omega_breve << endl
                     << endl; */
    }

    cout << "J matrix: " << endl
         << J << endl
         << endl;

    cout << "I_marg: " << endl
         << I_marg << endl
         << endl;

    /* J = J * U;
    cout << "J * U: " << endl
         << J << endl
         << endl; */

    // Eigen::MatrixXd Lambda = J * Sigma * J.transpose();
    Eigen::MatrixXd Lambda = I_marg;
    cout << "Lambda: " << endl
         << Lambda << endl
         << endl;

    for (int k = 0; k < edges_in_CLT.size(); ++k) {
        Eigen::MatrixXd non_zeros(DOF_3, DOF_3);
        non_zeros.fill(0);

        Eigen::MatrixXd J_k(DOF_3, J.row(0).size());
        J_k.fill(0);
        J_k = J.block(k * DOF_3, 0, DOF_3, J.row(0).size());
        cout << "J_k(k = " << k << "):" << endl
             << J_k << endl
             << endl;

        int k1 = edges_in_CLT[k][0] - 1;
        int k2 = edges_in_CLT[k][1] - 1;

        Eigen::MatrixXd J_k1(DOF_3, DOF_3);
        J_k1 = J_k.block(0, k1 * DOF_3, DOF_3, DOF_3);
        cout << "J_k1(k = " << k << "):" << endl
             << J_k1 << endl
             << endl;

        Eigen::MatrixXd J_k2(DOF_3, DOF_3);
        J_k2 = J_k.block(0, k2 * DOF_3, DOF_3, DOF_3);
        cout << "J_k2(k = " << k << "):" << endl
             << J_k2 << endl
             << endl;
        /*         int k1 = 0;
                int k2 = 0;

                Eigen::MatrixXd J_k1(DOF_3, DOF_3);
                J_k1.fill(0);

                Eigen::MatrixXd J_k2(DOF_3, DOF_3);
                J_k2.fill(0);

                J_k1 = J_k.block(0, k1 * DOF_3, DOF_3, DOF_3);
                while (J_k1 == non_zeros) {
                    ++k1;
                    J_k1 = J_k.block(0, k1 * DOF_3, DOF_3, DOF_3);
                }
                cout << "J_k1(k = " << k << "):" << endl
                     << J_k1 << endl
                     << endl;

                k2 = k1 + 1;
                J_k2 = J_k.block(0, k2 * DOF_3, DOF_3, DOF_3);
                while (J_k2 == non_zeros || k1 == k2) {
                    ++k2;
                    J_k2 = J_k.block(0, k2 * DOF_3, DOF_3, DOF_3);
                }
                cout << "J_k2(k = " << k << "):" << endl
                     << J_k2 << endl
                     << endl; */

        Eigen::MatrixXd Lambda_k1k2 = Lambda.block(k1 * DOF_3, k2 * DOF_3, DOF_3, DOF_3);
        cout << "Lambda_k1k2(k = " << k << "):" << endl
             << Lambda_k1k2 << endl
             << endl;

        Eigen::MatrixXd Omega_breve_k = J_k1.inverse().transpose() * Lambda_k1k2 * J_k2.inverse();
        cout << "Omega_breve_k(k = " << k << "):" << endl
             << Omega_breve_k << endl
             << endl;

        Omega_breve.block(k * DOF_3, k * DOF_3, DOF_3, DOF_3) = Omega_breve_k;
    }

    cout << "Initial guess Omega_breve: " << endl
         << Omega_breve << endl
         << endl;

    J = J * U;
    cout << "J * U: " << endl
         << J << endl
         << endl;

    // Factor Descent method
    {
        double threshold = 1;
        double epsilon = 0.0001;

        double current_KLD = 10;
        double pre_KLD = 1;
        double KLD_gradient = abs(current_KLD - pre_KLD) / pre_KLD;
        double KLD_gradient_norm = 10;

        int k = 0;
        Eigen::MatrixXd J_ks[edges_in_CLT.size()];
        Eigen::MatrixXd Phi_ks[edges_in_CLT.size()];
        Eigen::MatrixXd z_ks[edges_in_CLT.size()];
        for (int i = 0; i < edges_in_CLT.size(); ++i) {
            J_ks[i] = J.block(k * DOF_3, 0, DOF_3, J.row(0).size());
            cout << "J_ks[" << i << "]: " << endl
                 << J_ks[i] << endl
                 << endl;

            Phi_ks[i] = (J_ks[i] * Sigma * J_ks[i].transpose()).inverse();
            cout << "Phi_ks[" << i << "]: " << endl
                 << Phi_ks[i] << endl
                 << endl;

            z_ks[i] = z.block(0, k, DOF_3, 1);
            cout << "z_ks[" << i << "]: " << endl
                 << z_ks[i] << endl
                 << endl;
        }

        while (KLD_gradient_norm > threshold) {
            Eigen::MatrixXd Upsilon_k(Sigma.row(0).size(), Sigma.row(0).size());
            Upsilon_k.fill(0);
            for (int i = 0; i < edges_in_CLT.size(); ++i) {
                if (i != k) {
                    cout << "Omega_breve: " << endl
                         << Omega_breve << endl
                         << endl;

                    Eigen::MatrixXd Omega_breve_i = Omega_breve.block(i * DOF_3, i * DOF_3, DOF_3, DOF_3);
                    cout << "Omega_breve_i: " << endl
                         << Omega_breve_i << endl
                         << endl;

                    Upsilon_k += J_ks[i].transpose() * Omega_breve_i * J_ks[i];
                    cout << "Upsilon_k(updating): " << endl
                         << Upsilon_k << endl
                         << endl;
                }
            }

            cout << "Upsilon_k: " << endl
                 << Upsilon_k << endl
                 << endl;

            Eigen::MatrixXd Omega_breve_k = Phi_ks[k] - (J_ks[k] * Upsilon_k.inverse() * J_ks[k].transpose()).inverse();
            cout << "Omega Omega_breve_k values: " << endl
                 << Omega_breve_k << endl
                 << endl;

            Eigen::EigenSolver<Eigen::MatrixXd> eig_solver(Omega_breve_k);
            Eigen::MatrixXd Omega_breve_k_eig_mat = eig_solver.pseudoEigenvalueMatrix();
            cout << "Omega eigen values: " << endl
                 << Omega_breve_k_eig_mat << endl
                 << endl;

            Eigen::VectorXd Omega_breve_k_eig_diag = Omega_breve_k_eig_mat.diagonal();

            cout << "Omega_breve_k_eig_diag values: " << endl
                 << Omega_breve_k_eig_diag << endl
                 << endl;

            Eigen::MatrixXd V = eig_solver.pseudoEigenvectors();
            cout << "V(k = " << k << "):" << endl
                 << V << endl
                 << endl;

            bool is_positive_definite = true;

            for (int i = 0; i < Omega_breve_k_eig_diag.size(); ++i) {
                double eigen_value = Omega_breve_k_eig_diag(i, 0);
                if (eigen_value < epsilon) {
                    is_positive_definite = false;
                    Omega_breve_k_eig_diag(i, 0) = 0;
                }
            }
            cout << "Updated Omega_breve_k_eig_diag: " << endl
                 << Omega_breve_k_eig_diag << endl
                 << endl;

            if (is_positive_definite != true) {
                Omega_breve_k_eig_mat = Omega_breve_k_eig_diag.asDiagonal();
                Omega_breve_k = V * Omega_breve_k_eig_mat * V.transpose();
            }

            Omega_breve.block(k * DOF_3, k * DOF_3, DOF_3, DOF_3) = Omega_breve_k;
            cout << "Updated Omega_breve: " << endl
                 << Omega_breve << endl
                 << endl;

            Eigen::MatrixXd projected_I_spars = J.transpose() * Omega_breve * J;
            cout << "Updated projected_I_spars: " << endl
                 << projected_I_spars << endl
                 << endl;

            cout << "Sigma: " << endl
                 << Sigma << endl
                 << endl;

            /*             Eigen::MatrixXd prod = I_spars * Sigma;
                        cout << "prod: " << endl
                             << prod << endl
                             << endl;
                        pre_KLD = current_KLD;
                        double I_spars_determinant = abs(I_spars.determinant());
                        current_KLD = 0.5 * (prod.trace() - log(I_spars_determinant));

                        KLD_gradient = abs(current_KLD - pre_KLD) / pre_KLD;
                        cout << "KLD_gradient: " << KLD_gradient << endl;
          */

            Eigen::MatrixXd derivative_KLD = J_ks[k] * Sigma * J_ks[k].transpose() - J_ks[k] * (Upsilon_k + J_ks[k].transpose() * Omega_breve_k * J_ks[k]).inverse() * J_ks[k].transpose();
            cout << "derivative_KLD" << endl
                 << derivative_KLD << endl
                 << endl;

            double sum = 0;
            /*             for (int j = 0; j < J_ks[k].col(0).size(); ++j) {
                            Eigen::VectorXd col_vec = derivative_KLD.block(0, j, DOF_3, 1);
                            for (int i = 0; i < col_vec.size(); ++i) {
                                sum += abs(col_vec(i, 0));
                            }
                        } */
            for (int i = 0; i < derivative_KLD.col(0).size(); ++i) {
                for (int j = 0; j < derivative_KLD.row(0).size(); ++j) {
                    sum += derivative_KLD(i, j) * derivative_KLD(i, j);
                }
            }
            KLD_gradient_norm = sqrt(sum);
            cout << "KLD_gradient_norm: " << KLD_gradient_norm << endl
                 << endl;

            ++k;
            k %= edges_in_CLT.size();
        }
    }

    // Another method
    /*     {
            cout << "J before doing something: " << endl
                 << J << endl
                 << endl;

            cout << "Omega_breve before something: " << endl
                 << Omega_breve << endl
                 << endl;

            Eigen::MatrixXd J_k(DOF_3, I_marg.row(0).size());
            J_k.fill(0);
            J_k.block(0, 0, DOF_3, DOF_3) = Eigen::Matrix3d().Identity();
            // J_k.block(0, 0, DOF_3, DOF_3) = (Eigen::Vector3d(1, 1, 1)).asDiagonal();

            int last_idx_mb = I_marg.row(0).size() - DOF_3;
            Eigen::MatrixXd Omega_breve_k = (J_k * Sigma * J_k.transpose()).inverse();
            Omega_breve.block(last_idx_mb, last_idx_mb, DOF_3, DOF_3) = Omega_breve_k;
            cout << "Omega_breve after something: " << endl
                 << Omega_breve << endl
                 << endl;

            J.block(last_idx_mb, 0, DOF_3, J.row(0).size()) = J_k;
            cout << "J after doing something: " << endl
                 << J << endl
                 << endl;
        } */

    Eigen::MatrixXd projected_I_spars = J.transpose() * Omega_breve * J;
    Eigen::MatrixXd I_spars = U * projected_I_spars * U.transpose();
    cout << "I_spars: " << endl
         << I_spars << endl
         << endl;

    ofstream os3("sparcification.txt");
    os3 << I_spars << endl;
    os3.close();

    // To get Kullback-Leibler divergence
    cout << "projected_I_spars's row size: " << projected_I_spars.row(0).size() << endl;
    cout << "projected_I_spars's column size: " << projected_I_spars.col(0).size() << endl;

    cout << "Sigma's row size: " << Sigma.row(0).size() << endl;
    cout << "Sigma's column size: " << Sigma.col(0).size() << endl;

    Eigen::Product prod = projected_I_spars * Sigma;

    // Eigen::Product prod = I_spars * Sigma;
    double prod_det = prod.determinant();
    cout << "prod.trace: " << prod.trace() << endl;
    cout << "prod.det: " << prod_det << endl;

    double KLD = 0.5 * (prod.trace() - log(prod_det) - projected_I_spars.row(0).size());
    cout << "KLD: " << KLD << endl
         << endl;

    // To remove nodes to create a sparsed graph
    for (int node_to_remove : nodes_to_remove) {
        for (auto it = original_graph.begin(); it != original_graph.end(); ++it) {
            auto node_pair = (*it)->keys();
            if (node_pair.size() != 1) {
                if (node_pair[0] == node_to_remove || node_pair[1] == node_to_remove || (markov_blanket.exists(node_pair[0]) == true && markov_blanket.exists(node_pair[1]) == true)) {
                    original_graph.erase(it);
                    --it;
                }
            }
        }
        original_initial.erase(node_to_remove);
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

        Eigen::MatrixXd new_rel_pose = z.block(0, k, DOF_3, 1);
        double new_t_x = new_rel_pose(0, 0);
        double new_t_y = new_rel_pose(1, 0);
        double new_t_theta = new_rel_pose(2, 0);

        double correct_theta_result = result.at<Pose2>(j_key).theta();
        new_t_theta = abs(correct_theta_result - (pi.theta() + new_t_theta)) < abs(correct_theta_result - (pi.theta() - new_t_theta)) ? new_t_theta : -new_t_theta;

        cout << "new transition x: " << new_t_x << endl;
        cout << "new transition y: " << new_t_y << endl;
        cout << "new transition theta: " << new_t_theta << endl
             << endl;

        Eigen::MatrixXd cov = Omega_breve.block(k * DOF_3, k * DOF_3, DOF_3, DOF_3).inverse();
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

        normal_distribution<double> error_x_distribution(0.0, x_cov);
        normal_distribution<double> error_y_distribution(0.0, y_cov);
        normal_distribution<double> error_theta_distribution(0.0, theta_cov);
        double error_x = error_x_distribution(generator);
        double error_y = error_y_distribution(generator);
        double error_theta = error_theta_distribution(generator);

        double pj_x = cos(pi.theta()) * (new_t_x + error_x) - sin(pi.theta()) * (new_t_y + error_y) + pi.x();
        double pj_y = sin(pi.theta()) * (new_t_x + error_x) + cos(pi.theta()) * (new_t_y + error_y) + pi.y();
        double pj_theta = new_t_theta + pi.theta() + error_theta;

        cout << "j_key: " << j_id << endl;
        cout << "pj_x: " << pj_x << endl;
        cout << "pj_y: " << pj_y << endl;
        cout << "pj_theta: " << pj_theta << endl
             << endl;

        auto sparsified_noise = noiseModel::Diagonal::Sigmas(Vector3(sqrt(x_cov), sqrt(y_cov), sqrt(theta_cov)));
        // original_graph.emplace_shared<BetweenFactor<Pose2>>(i_key, j_key, Pose2(new_t_x, new_t_y, new_t_theta), sparsified_noise);
        original_graph.add(BetweenFactor<Pose2>(i_key, j_key, Pose2(new_t_x, new_t_y, new_t_theta), sparsified_noise));
        original_initial.update(j_key, Pose2(pj_x, pj_y, pj_theta));
    }

    result = LevenbergMarquardtOptimizer(original_graph, original_initial).optimize();

    ofstream os4("sparsified_coords.txt");
    for (int i = 1; i < original_graph.size(); ++i) {
        auto factor = original_graph.at(i);
        auto keys = factor->keys();

        auto src = keys[0];
        auto dest = keys[1];

        Pose2 p1 = result.at<Pose2>(src);
        Pose2 p2 = result.at<Pose2>(dest);

        Pose2 i1 = original_initial.at<Pose2>(src);
        Pose2 i2 = original_initial.at<Pose2>(dest);

        os4 << p1.x() << " " << p1.y() << " " << p2.x() << " " << p2.y() << " " << i1.x() << " " << i1.y() << " " << i2.x() << " " << i2.y() << std::endl;
    }
    os4.close();

    return 0;
}
