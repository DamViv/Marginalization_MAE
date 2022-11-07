#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

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

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
    vector<vector<double>> odometries;
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

    // ---------------------------- Try 1         -------------------------------------

    // Create Map of TimeStamp
    map<string, int> time_stamps_map;
    for (int i = 0; i < odometries.size(); ++i) {
        time_stamps_map.insert(pair<string, int>(to_string(odometries[i][0]), 0));
        time_stamps_map.insert(pair<string, int>(to_string(odometries[i][1]), 0));
    }

    // Generate a graph
    size_t i = 1;

    NonlinearFactorGraph graph;

    Pose2 priorMean(0.0, 0.0, 0.0);

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0, 0, 0));
    graph.addPrior(i, priorMean, priorNoise);

    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0, 0, 0));

    Values initial;
    vector<double> pose = {0, 0, 0};
    double theta = pose[2];

    initial.insert(i, Pose2(pose[0], pose[1], pose[2]));

    vector<vector<double>>::iterator iter = odometries.begin();

    // time_stamp = {time_stamp1, time_stamp2}
    vector<double> time_stamp = {(*iter)[0], (*iter)[1]};

    // transition = {x, y, theta}
    vector<double> transition = {(*iter)[2], (*iter)[3], (*iter)[4]};

    // add factor between two poses
    Pose2 odometry(transition[0], transition[1], transition[2]);
    graph.emplace_shared<BetweenFactor<Pose2>>(i, i + 1, odometry, odometryNoise);

    // Initialize one pose
    pose[0] = pose[0] * cos(transition[2]) - pose[1] * sin(transition[2]) + transition[0];
    pose[1] = pose[0] * sin(transition[2]) + pose[1] * cos(transition[2]) + transition[1];
    pose[2] += transition[2];

    pose[2] = pose[2] < -2 * M_PI ? pose[2] + 2 * M_PI : pose[2];
    pose[2] = pose[2] > 2 * M_PI ? pose[2] - 2 * M_PI : pose[2];

    initial.insert(i + 1, Pose2(pose[0], pose[1], pose[2]));

    // Count number of visits
    time_stamps_map[to_string(time_stamp[0])] = 1;
    time_stamps_map[to_string(time_stamp[1])] = 1;

    map<string, int>::iterator iter_map = time_stamps_map.end();
    map<string, int>::iterator last_iter_map = time_stamps_map.find(to_string(odometries[odometries.size() - 1][1]));

    while ((last_iter_map->second) < 1) {
        ++i;
        if (i = 755) {
            cout << "Stop" << endl;
        }

        iter = odometries.begin();
        while (iter != odometries.end()) {
            if (abs((*iter)[0] - time_stamp[1]) <= __DBL_EPSILON__) {
                break;
            } else {
                ++iter;
            }
        }

        if (iter == odometries.end()) {
            double ts1 = stod(iter_map->first);
            iter_map++;
            double ts2 = stod(iter_map->first);

            vector<double> tmp = {ts1, ts2, 0, 0, 0};
            odometries.push_back(tmp);
            sort(odometries.begin(), odometries.end());

            iter = odometries.begin();
            while (iter != odometries.end()) {
                if (abs((*iter)[0] - time_stamp[1]) <= __DBL_EPSILON__) {
                    break;
                } else {
                    ++iter;
                }
            }
        }

        iter_map = time_stamps_map.find(to_string((*iter)[0]));

        if ((iter_map->second) > 1) {
            iter += (iter_map->second) - 1;
            cout << "Loop closing!" << endl;
        }

        // time_stamp = {time_stamp1, time_stamp2}
        time_stamp = {(*iter)[0], (*iter)[1]};

        // transition = {x, y, theta}
        transition = {(*iter)[2], (*iter)[3], (*iter)[4]};

        // add factor between two poses
        Pose2 odometry(transition[0], transition[1], transition[2]);
        graph.emplace_shared<BetweenFactor<Pose2>>(i, i + 1, odometry, odometryNoise);

        // Initialize one pose
        pose[0] = pose[0] * cos(transition[2]) - pose[1] * sin(transition[2]) + transition[0];
        pose[1] = pose[0] * sin(transition[2]) + pose[1] * cos(transition[2]) + transition[1];
        pose[2] += transition[2];

        pose[2] = pose[2] < -2 * M_PI ? pose[2] + 2 * M_PI : pose[2];
        pose[2] = pose[2] > 2 * M_PI ? pose[2] - 2 * M_PI : pose[2];

        initial.insert(i + 1, Pose2(pose[0], pose[1], pose[2]));

        // Count number of visits
        iter_map = time_stamps_map.find(to_string(time_stamp[1]));
        iter_map->second = iter_map->second + 1;
    }

    // ---------------------------------------------------------------------------------------
    // ---------------------------- Initial try          -------------------------------------
    /*
        // Generate a graph
        NonlinearFactorGraph graph;

        Pose2 priorMean(0.0, 0.0, 0.0);

        auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0, 0, 0));
        graph.addPrior(1, priorMean, priorNoise);

        auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0, 0, 0));

        Values initial;
        vector<double> pose = {0, 0, 0};
        double theta = pose[2];

        size_t i = 0;
        initial.insert(i + 1, Pose2(pose[0], pose[1], pose[2]));

        for (vector<double> transition : odometries) {
            Pose2 odometry(transition[0], transition[1], transition[2]);
            graph.emplace_shared<BetweenFactor<Pose2>>(i + 1, i + 2, odometry, odometryNoise);

            pose[0] = pose[0] * cos(transition[2]) - pose[1] * sin(transition[2]) + transition[0];
            pose[1] = pose[0] * sin(transition[2]) + pose[1] * cos(transition[2]) + transition[1];
            pose[2] += transition[2];

            pose[2] = pose[2] < -2 * M_PI ? pose[2] + 2 * M_PI : pose[2];
            pose[2] = pose[2] > 2 * M_PI ? pose[2] - 2 * M_PI : pose[2];

            theta = pose[2];

            i++;
            initial.insert(i + 1, Pose2(pose[0], pose[1], pose[2]));
        } */
    // ---------------------------------------------------------------------------------------

    // Single Step Optimization using Levenberg-Marquardt
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    //
    // result.print("Final Result:\n");

    // save factor graph as graphviz dot file
    // Render to PDF using "fdp Pose2SLAM_result.dot -Tpdf > graph.pdf"
    graph.saveGraph("Pose2SLAM_result.dot", result);

    //  Also print out to console
    // graph.dot(cout, result);

    return 0;
}