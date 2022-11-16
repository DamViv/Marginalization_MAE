#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

#include "file_utils.h"
#include "matplotlibcpp.h"
#include "point.h"

using namespace std;
using namespace gtsam;
namespace plt = matplotlibcpp;

int main(int argc, char** argv) {
    vector<vector<double>> odometries;  // odometries: [start_time_stamp, end_time_stamp, x, y, theta]
    odometries.reserve(3000);

    // string file_name = "test.txt";
    string file_name = "intel.relations.txt";

    string basic_path = "./data_set/";
    string data_path = basic_path + file_name;

    // Read text file
    extract_data(data_path, odometries);

    // Sort the data in ascending the time stamp
    sort(odometries.begin(), odometries.end());

    // Write the sorted data in text file
    file_name = "sorted_data.txt";
    data_path = basic_path + file_name;
    write_data(data_path, odometries);

    // Make a set of pose_id in a Hash map and the one of pose_info
    // pose_info: [time_stamp, {index_number, visit_time, x, y, theta}]
    map<string, vector<double>> pose_info;

    // Initialize pose_info
    for (int i = 0; i < odometries.size(); ++i) {
        pose_info.insert(pair<string, vector<double>>(to_string(odometries[i][0]), {0, 0, 0, 0}));
        pose_info.insert(pair<string, vector<double>>(to_string(odometries[i][1]), {0, 0, 0, 0}));
    }

    size_t pose_index = 1;
    for (map<string, vector<double>>::iterator iter = pose_info.begin(); iter != pose_info.end(); ++iter) {
        (iter->second)[0] = pose_index;
        ++pose_index;
    }

    // Generate a graph
    pose_index = 1;
    NonlinearFactorGraph graph;

    Pose2 priorMean(0.0, 0.0, 0.0);

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    graph.addPrior(pose_index, priorMean, priorNoise);

    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 0.1));

    // Generate the initial
    Values initial;

    initial.insert(pose_index, Pose2(0, 0, 0));

    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    size_t i = 0;
    string start = to_string(odometries[i][0]);
    string end = "";

    {
        map<string, vector<double>>::iterator iter = pose_info.find(start);
        assert(iter != pose_info.end() && "Couldn't find the first time stamp!");
        (iter->second)[1] = 1;
    }

    // Add graph and perform initial estimations
    bool is_circulated = false;

    while (odometries.size() != 0) {
        start = to_string(odometries[i][0]);
        end = to_string(odometries[i][1]);

        map<string, vector<double>>::iterator start_info_it = pose_info.find(start);
        map<string, vector<double>>::iterator end_info_it = pose_info.find(end);

        assert(start_info_it != pose_info.end() && "the start pose doesn't exist in pose_info.");
        assert(end_info_it != pose_info.end() && "the end pose doesn't exist in pose_info.");

        int start_idx = (start_info_it->second)[0];
        int end_idx = (end_info_it->second)[0];

        int num_visits_start = (start_info_it->second)[1];
        int num_visits_end = (end_info_it->second)[1];

        if (num_visits_start >= 1) {  // When the start pose is linked into its previous pose.

            // odometries: [start_time_stamp, end_time_stamp, x, y, theta] in relataive coordinates
            // transition: [x, y, theta] in relative coordinates
            vector transition = {odometries[i][2], odometries[i][3], odometries[i][4]};
            Pose2 odometry(transition[0], transition[1], transition[2]);

            graph.emplace_shared<BetweenFactor<Pose2>>(start_idx, end_idx, odometry, odometryNoise);

            // pose_info: [[time_stamp, {index_number, visit_time, x, y, theta}]]
            double start_x = (start_info_it->second)[2];
            double start_y = (start_info_it->second)[3];
            double start_theta = (start_info_it->second)[4];

            double end_x = cos(start_theta) * transition[0] - sin(start_theta) * transition[1] + start_x;
            double end_y = sin(start_theta) * transition[0] + cos(start_theta) * transition[1] + start_y;
            double end_theta = start_theta + transition[2];

            end_theta = end_theta < -2 * M_PI ? end_theta + 2 * M_PI : end_theta;
            end_theta = end_theta > 2 * M_PI ? end_theta - 2 * M_PI : end_theta;

            (end_info_it->second)[1] += 1;
            (end_info_it->second)[2] = end_x;
            (end_info_it->second)[3] = end_y;
            (end_info_it->second)[4] = end_theta;

            if (num_visits_end == 0) {  // When the end pose is the first visit

                initial.insert(end_idx, Pose2(end_x, end_y, end_theta));
            } else {  // When the end pose is already visited
                initial.update(end_idx, Pose2(end_x, end_y, end_theta));
            }

            odometries.erase(odometries.begin() + i);
        } else if (num_visits_end >= 1) {
            // When the start pose is not linked into its previous pose but the end pose is already visited.

            // odometries: [start_time_stamp, end_time_stamp, x, y, theta] in relataive coordinates
            // transition: [x, y, theta] in relative coordinates
            vector transition = {odometries[i][2], odometries[i][3], odometries[i][4]};
            Pose2 odometry(transition[0], transition[1], transition[2]);

            graph.emplace_shared<BetweenFactor<Pose2>>(start_idx, end_idx, odometry, odometryNoise);

            // pose_info: [[time_stamp, {index_number, visit_time, x, y, theta}]]
            // Calculate the start pose in absolute coordinates
            double end_x = (end_info_it->second)[2];
            double end_y = (end_info_it->second)[3];
            double end_theta = (end_info_it->second)[4];

            double start_theta = end_theta - transition[2];
            start_theta = start_theta < -2 * M_PI ? start_theta + 2 * M_PI : start_theta;
            start_theta = start_theta > 2 * M_PI ? start_theta - 2 * M_PI : start_theta;

            double start_x = end_x - cos(start_theta) * transition[0] + sin(start_theta) * transition[1];
            double start_y = end_y - sin(start_theta) * transition[0] - cos(start_theta) * transition[1];

            // Update start pose information
            (start_info_it->second)[1] += 1;  // Increase the number of visits
            (start_info_it->second)[2] = start_x;
            (start_info_it->second)[3] = start_y;
            (start_info_it->second)[4] = start_theta;

            // Get initial estimation
            initial.insert(start_idx, Pose2(start_x, start_y, start_theta));

            odometries.erase(odometries.begin() + i);
        } else if (num_visits_start == 0 && num_visits_end == 0 && is_circulated == true) {
            // odometries: [start_time_stamp, end_time_stamp, x, y, theta] in relataive coordinates
            // transition: [x, y, theta] in relative coordinates
            vector transition = {odometries[i][2], odometries[i][3], odometries[i][4]};
            Pose2 odometry(transition[0], transition[1], transition[2]);

            graph.emplace_shared<BetweenFactor<Pose2>>(start_idx, end_idx, odometry, odometryNoise);

            (start_info_it->second)[1] += 1;
            initial.insert(start_idx, Pose2(0, 0, 0));

            (end_info_it->second)[1] += 1;
            (end_info_it->second)[2] = transition[0];
            (end_info_it->second)[3] = transition[1];
            (end_info_it->second)[4] = transition[2];
            initial.insert(end_idx, Pose2(transition[0], transition[1], transition[2]));

        } else {
            // When both start and end poses aren't linked to anything, skip this process and just go to the next step.
            // It will be handled afterwards

            ++i;
            if (i >= odometries.size() - 1) {
                i = 0;
            }
            continue;
        }

        if (i == odometries.size()) {
            is_circulated = true;
            i = 0;
        }

        // Perform Optimization at each step using Levenberg-Marquardt
        result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    }

    // Check if all odometries are input in graph
    assert(odometries.size() == 0 && "The set of odomotries isn't empty! It should be empty.");

    // print result
    // result.print("Final Result:\n");

    // save factor graph as graphviz dot file
    // Render to PDF using "fdp Pose2_SLAM_result.dot -Tpdf > Pose2_SLAM_result.pdf"
    file_name = "Pose2_SLAM_result.dot";
    data_path = file_name;
    graph.saveGraph(file_name, result);

    vector<Point> poses;
    poses.reserve(3000);

    extract_poses(data_path, poses);

    vector<float> pose_x;
    pose_x.reserve(poses.size());

    vector<float> pose_y;
    pose_y.reserve(poses.size());

    for (int i = 0; i < poses.size(); ++i) {
        pose_x.push_back(poses[i].mX);
        pose_y.push_back(poses[i].mY);
    }

    plt::figure_size(1200, 780);
    plt::scatter(pose_x, pose_y);
    plt::title("Seonghyun's Result");
    plt::save("./Result.png");

    plt::show();

    //  Also print out to console
    // graph.dot(cout, result);

    return 0;
}
