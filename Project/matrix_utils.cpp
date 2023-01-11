#include "matrix_utils.h"

#include <iostream>
using namespace std;

void computeMatInfJac(gtsam::Matrix& I_markov, gtsam::NonlinearFactorGraph& graph, gtsam::Values& result, gtsam::noiseModel::Diagonal::shared_ptr& odom_noise_model, gtsam::KeySet& markov_blanket) {
    int dim_lambda = I_markov.row(0).size();

    for (int k = 1; k < graph.size(); ++k) {
        auto keys = graph.at(k)->keys();
        int i = keys[0];
        int j = keys[1];
        if (markov_blanket.find(i) == markov_blanket.end() || markov_blanket.find(j) == markov_blanket.end()) {
            continue;
        }
        Eigen::MatrixXd I_ij = odom_noise_model->information();
        cout << "I_ij: " << endl
             << I_ij << endl
             << endl;

        gtsam::Pose2 meas_i = result.at<gtsam::Pose2>(i);
        cout << "i node number: " << i << endl;

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

        gtsam::Pose2 meas_j = result.at<gtsam::Pose2>(j);
        cout << "j node number: " << j << endl;

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

        int p = 0;
        int q = 0;
        int idx = 0;
        for (auto node : markov_blanket) {
            if (node == i) {
                p = idx;
            }

            if (node == j) {
                q = idx;
            }
            ++idx;
        }

        Eigen::MatrixXd H(DOF_3, dim_lambda);
        H.fill(0);

        Eigen::MatrixXd tmp_matrix(DOF_3, DOF_3);
        tmp_matrix.fill(0);
        tmp_matrix.block(0, 0, DOF_3 - 1, DOF_3 - 1) = Rj.transpose() * Ri;
        tmp_matrix.block(0, DOF_3 - 1, DOF_3 - 1, 1) = R_perp * Rj.transpose() * (ti - tj);
        Eigen::Matrix<double, 1, 3> block_row = {0, 0, 1};
        tmp_matrix.block(DOF_3 - 1, 0, 1, DOF_3) = block_row;
        H.block(0, p * DOF_3, DOF_3, DOF_3) = -tmp_matrix;
        H.block(0, q * DOF_3, DOF_3, DOF_3) = Eigen::Matrix3d().Identity();

        I_markov = I_markov + H.transpose() * I_ij * H;
    }
}

void compute_information_markov(gtsam::Matrix& I_markov, gtsam::JointMarginal& I_joint, gtsam::KeySet& markov_blanket) {
    int offset_i = 0;
    int offset_j = 0;

    for (auto i : markov_blanket) {
        for (auto j : markov_blanket) {
            gtsam::Matrix I_partial = I_joint.at(i, j);
            cout << "I_partial: " << endl
                 << I_partial << endl
                 << endl;
            I_markov.block(offset_i, offset_j, DOF_3, DOF_3) = I_partial;
            cout << "I_markov(creating): " << endl
                 << I_markov << endl
                 << endl;
            offset_j += DOF_3;
        }
        offset_j = 0;
        offset_i += DOF_3;
    }
}

void compute_marginalized_informaiton_markov(Eigen::MatrixXd& I_marg, gtsam::Matrix& I_markov, gtsam::KeySet& nodes_to_keep_mb, std::unordered_map<int, int>& map_index_t, std::unordered_map<int, int>& map_node_t) {
    int I_markov_row_length = I_markov.row(0).size();
    int marginalized_length = nodes_to_keep_mb.size() * DOF_3;

    int n_keep_mb = nodes_to_keep_mb.size();
    int n_remove_mb = (I_markov_row_length / DOF_3) - n_keep_mb;
    int k = 1;

    for (auto node : nodes_to_keep_mb) {
        int i = map_node_t[node];
        // Exchange rows
        Eigen::MatrixXd rows_k = I_markov.block((k - 1) * 3, 0, 3, I_markov_row_length);
        Eigen::MatrixXd rows_i = I_markov.block((i - 1) * 3, 0, 3, I_markov_row_length);
        I_markov.block((k - 1) * 3, 0, 3, I_markov_row_length) = rows_i;
        I_markov.block((i - 1) * 3, 0, 3, I_markov_row_length) = rows_k;

        // Exchange columns
        Eigen::MatrixXd cols_k = I_markov.block(0, (k - 1) * 3, I_markov_row_length, 3);
        Eigen::MatrixXd cols_i = I_markov.block(0, (i - 1) * 3, I_markov_row_length, 3);
        I_markov.block(0, (k - 1) * 3, I_markov_row_length, 3) = cols_i;
        I_markov.block(0, (i - 1) * 3, I_markov_row_length, 3) = cols_k;

        // Update maps
        map_index_t[k] = node;
        map_index_t[i] = map_index_t[k];

        map_node_t[node] = k;
        map_node_t[map_index_t[k]] = i;
        ++k;
    }

    cout << "I_markov after re-oreder: " << endl
         << I_markov << endl
         << endl;

    gtsam::Matrix I_ii(n_keep_mb * DOF_3, n_keep_mb * DOF_3);
    I_ii.block(0, 0, n_keep_mb * DOF_3, n_keep_mb * DOF_3) = I_markov.block(0, 0, n_keep_mb * DOF_3, n_keep_mb * DOF_3);
    cout << "I_ii: \n"
         << I_ii << endl
         << endl;

    gtsam::Matrix I_ij(n_keep_mb * DOF_3, n_remove_mb * DOF_3);
    I_ij.block(0, 0, n_keep_mb * DOF_3, n_remove_mb * DOF_3) = I_markov.block(0, n_keep_mb * DOF_3, n_keep_mb * DOF_3, n_remove_mb * DOF_3);

    cout << "I_ij: \n"
         << I_ij << endl
         << endl;

    gtsam::Matrix I_jj = I_markov.block(n_keep_mb * DOF_3, n_keep_mb * DOF_3, n_remove_mb * DOF_3, n_remove_mb * DOF_3);
    cout << "I_jj: \n"
         << I_jj << endl
         << endl;

    I_marg = I_ii - I_ij * ((I_jj.completeOrthogonalDecomposition()).pseudoInverse()) * I_ij.transpose();
}

double compute_mutual_information(Eigen::MatrixXd& I, int i, int j) {
    Eigen::MatrixXd I_joint = compute_joint_marginal(I, i, j);
    cout << "I_joint: " << endl
         << I_joint << endl
         << endl;

    Eigen::MatrixXd I_ii = I_joint.block(0, 0, DOF_3, DOF_3);
    cout << "I_ii: " << endl
         << I_ii << endl
         << endl;

    Eigen::MatrixXd I_ij = I_joint.block(0, DOF_3, DOF_3, DOF_3);
    cout << "I_ij: " << endl
         << I_ij << endl
         << endl;

    Eigen::MatrixXd I_jj = I_joint.block(DOF_3, DOF_3, DOF_3, DOF_3);
    cout << "I_jj: " << endl
         << I_jj << endl
         << endl;

    double numerator = (I_ii + Eigen::Matrix3d::Identity()).determinant();
    double denominator = (I_ii - (I_ij * (I_jj.completeOrthogonalDecomposition()).pseudoInverse() * I_ij.transpose()) + Eigen::Matrix3d::Identity()).determinant();

    double MI = 0.5 * log(numerator / denominator);

    return MI;
}

Eigen::MatrixXd compute_joint_marginal(Eigen::MatrixXd I, int i, int j) {
    int row_length = I.row(0).size();
    int n = row_length / DOF_3;

    if (n == 2) {
        cout << "I: " << endl
             << I << endl
             << endl;
        return I;
    }

    int k = 1;
    Eigen::MatrixXd rows_k = I.block((k - 1) * DOF_3, 0, DOF_3, row_length);
    Eigen::MatrixXd rows_i = I.block((i - 1) * DOF_3, 0, DOF_3, row_length);
    cout << "k = 1: " << endl;
    cout << "rows_k: " << endl
         << rows_k << endl
         << endl;

    cout << "rows_i: " << endl
         << rows_i << endl
         << endl;

    I.block((k - 1) * DOF_3, 0, DOF_3, row_length) = rows_i;
    I.block((i - 1) * DOF_3, 0, DOF_3, row_length) = rows_k;

    cout << "I(after row changed, k = 1): " << endl
         << I << endl
         << endl;

    Eigen::MatrixXd cols_k = I.block(0, (k - 1) * DOF_3, row_length, DOF_3);
    Eigen::MatrixXd cols_i = I.block(0, (i - 1) * DOF_3, row_length, DOF_3);

    cout << "cols_k: " << endl
         << cols_k << endl
         << endl;

    cout << "cols_i: " << endl
         << cols_i << endl
         << endl;

    I.block(0, (k - 1) * DOF_3, row_length, DOF_3) = cols_i;
    I.block(0, (i - 1) * DOF_3, row_length, DOF_3) = cols_k;

    cout << "I(after column changed, k = 1): " << endl
         << I << endl
         << endl;

    k = 2;
    rows_k = I.block((k - 1) * DOF_3, 0, DOF_3, row_length);
    Eigen::MatrixXd rows_j = I.block((j - 1) * DOF_3, 0, DOF_3, row_length);

    cout << "k = 2: " << endl;
    cout << "rows_k: " << endl
         << rows_k << endl
         << endl;

    cout << "rows_j: " << endl
         << rows_j << endl
         << endl;

    I.block((k - 1) * DOF_3, 0, DOF_3, row_length) = rows_j;
    I.block((j - 1) * DOF_3, 0, DOF_3, row_length) = rows_k;

    cout << "I(after row changed, k = 2): " << endl
         << I << endl
         << endl;

    cols_k = I.block(0, (k - 1) * DOF_3, row_length, DOF_3);
    Eigen::MatrixXd cols_j = I.block(0, (j - 1) * DOF_3, row_length, DOF_3);

    cout << "cols_k: " << endl
         << cols_k << endl
         << endl;

    cout << "cols_j: " << endl
         << cols_j << endl
         << endl;

    I.block(0, (k - 1) * DOF_3, row_length, DOF_3) = cols_j;
    I.block(0, (j - 1) * DOF_3, row_length, DOF_3) = cols_k;

    cout << "I(after column changed, k = 2): " << endl
         << I << endl
         << endl;

    Eigen::MatrixXd I_aa = I.block(0, 0, DOF_3 * 2, DOF_3 * 2);
    Eigen::MatrixXd I_bb = I.block(DOF_3 * 2, DOF_3 * 2, row_length - DOF_3 * 2, row_length - DOF_3 * 2);
    Eigen::MatrixXd I_ab = I.block(0, DOF_3 * 2, DOF_3 * 2, row_length - DOF_3 * 2);

    Eigen::MatrixXd I_joint = I_aa - I_ab * (I_bb.completeOrthogonalDecomposition()).pseudoInverse() * I_ab.transpose();

    cout << "I_aa: " << endl
         << I_aa << endl
         << endl;

    cout << "I_ab: " << endl
         << I_ab << endl
         << endl;

    cout << "I_bb: " << endl
         << I_bb << endl
         << endl;

    cout << "I_joint: " << endl
         << I_joint << endl
         << endl;

    return I_joint;
}