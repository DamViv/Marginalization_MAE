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

    string file_name = "test.txt";
    // string file_name = "intel.relations.txt";

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

    // Generate a graph
    size_t pose_index = 1;
    NonlinearFactorGraph graph;

    Pose2 priorMean(0.0, 0.0, 0.0);

    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0, 0, 0));
    graph.addPrior(pose_index, priorMean, priorNoise);

    auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0, 0, 0));

    // Generate the initial
    Values initial;
    // vector<double> initial_pose = {0, 0, 0};

    initial.insert(pose_index, Pose2(0, 0, 0));

    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    // time_stamps_index: [time_stamp, pose_index]
    map<string, int> time_stamps_index;

    // travel_info: [time_stamp, [visit_time, x, y, theta]]
    map<string, vector<double>> travel_info;

    // Initialize time_stamps_index and travel_info
    for (int i = 0; i < odometries.size(); ++i) {
        time_stamps_index.insert(pair<string, int>(to_string(odometries[i][0]), pose_index));
        travel_info.insert(pair<string, vector<double>>(to_string(odometries[i][0]), {0, 0, 0, 0}));
        ++pose_index;
    }

    // Run
    for (int i = 0; i < odometries.size(); ++i) {
        if (i == 755) {
            break;
        }

        string provenance = to_string(odometries[i][0]);
        string destination = to_string(odometries[i][1]);
        int provenance_index = 0;
        int destination_index = 0;

        // Add factor
        map<string, int>::iterator iter_index = time_stamps_index.find(provenance);
        if (iter_index != time_stamps_index.end()) {
            provenance_index = iter_index->second;
        } else {
            cout << "The provenance " << provenance << " doesn't exist in map" << endl;
            break;
        }

        iter_index = time_stamps_index.find(destination);
        if (iter_index != time_stamps_index.end()) {
            destination_index = iter_index->second;
        } else {
            int max_index = -1;
            for (iter_index = time_stamps_index.begin(); iter_index != time_stamps_index.end(); ++iter_index) {
                max_index = (iter_index->second) > max_index ? (iter_index->second) : max_index;
            }
            destination_index = max_index + 1;
            time_stamps_index[destination] = destination_index;
            travel_info[destination] = {0, 0, 0, 0};
        }

        // transition = {x, y, theta}
        vector transition = {odometries[i][2], odometries[i][3], odometries[i][4]};
        Pose2 odometry(transition[0], transition[1], transition[2]);
        graph.emplace_shared<BetweenFactor<Pose2>>(provenance_index, destination_index, odometry, odometryNoise);

        // Estimate initialization
        // travel_info: [time_stamp, [visit_time, x, y, theta]]
        map<string, vector<double>>::iterator iter_travel_dst = travel_info.find(destination);
        if ((iter_travel_dst->second)[0] != 0) {
            continue;
        }

        map<string, vector<double>>::iterator iter_travel_pro = travel_info.find(provenance);
        double theta = (iter_travel_pro->second)[3];
        double destination_x = cos(theta) * odometries[i][2] - sin(theta) * odometries[i][3] + (iter_travel_pro->second)[1];
        double destination_y = sin(theta) * odometries[i][2] + cos(theta) * odometries[i][3] + (iter_travel_pro->second)[2];
        double destination_theta = theta + odometries[i][4];

        destination_theta = destination_theta < -2 * M_PI ? destination_theta + 2 * M_PI : destination_theta;
        destination_theta = destination_theta > 2 * M_PI ? destination_theta - 2 * M_PI : destination_theta;

        (iter_travel_dst->second)[0] += 1;
        (iter_travel_dst->second)[1] = destination_x;
        (iter_travel_dst->second)[2] = destination_y;
        (iter_travel_dst->second)[3] = destination_theta;

        initial.insert(destination_index, Pose2(destination_x, destination_y, destination_theta));

        // Single Step Optimization using Levenberg-Marquardt
        result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    }

    // print result
    // result.print("Final Result:\n");

    // save factor graph as graphviz dot file
    // Render to PDF using "fdp Pose2SLAM_result.dot -Tpdf > graph.pdf"
    graph.saveGraph("Pose2SLAM_result.dot", result);

    //  Also print out to console
    // graph.dot(cout, result);

    return 0;
}