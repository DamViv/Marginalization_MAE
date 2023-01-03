#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "file_utils.h"
#include "node.h"
#include "relation.h"

using namespace std;
using namespace gtsam;

bool compare(relation a, relation b) {
    if (a.id1 == b.id2) {
        return a.id2 < b.id2;
    } else {
        return a.id1 < b.id2;
    }
}

int main(int argc, char** argv) {
    string relation_file = "intel.relations.txt";
    string basic_path = "./data_set/";

    relation_file = basic_path + relation_file;
    std::list<relation> relations;
    /*     std::vector<relation> relations;
        relations.reserve(3000); */

    loadGraph_from_relations(relation_file, relations);

    // sort(relations.begin(), relations.end(), compare);

    // Initialize graph with prior on first pose
    NonlinearFactorGraph graph;
    Values initial;

    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
    noiseModel::Diagonal::shared_ptr odom_noise_model = noiseModel::Diagonal::Sigmas(Vector3(1, 1, 0.1));

    // Create a map to link time and key
    std::map<string, unsigned int> existing_nodes;

    // Add the first node with key 0 with a Prior factor to fix the traj
    unsigned int key = 0;
    graph.add(PriorFactor<Pose2>(key, Pose2(0, 0, 0), priorNoise));
    initial.insert(key, Pose2(0, 0, 0));
    list<relation>::iterator it = relations.begin();
    existing_nodes.insert(pair<string, unsigned int>(it->id1, key));

    // Perform Optimization at each step using Levenberg-Marquardt
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    int count = 0;
    for (relation rel : relations) {
        /*         if (count > 2200) {
                    break;
                } */
        string src_node = rel.id1;
        string dest_node = rel.id2;

        // check if source node exists in exising_nodes
        if (existing_nodes.find(src_node) != existing_nodes.end()) {
            Pose2 src_pose = initial.at<Pose2>(existing_nodes[src_node]);
            double x = (cos(src_pose.theta()) * rel.T.x() - sin(src_pose.theta()) * rel.T.y()) + src_pose.x();
            double y = (sin(src_pose.theta()) * rel.T.x() + cos(src_pose.theta()) * rel.T.y()) + src_pose.y();
            double theta = src_pose.theta() + rel.R.z();

            // check if destination node exists
            if (existing_nodes.find(dest_node) != existing_nodes.end()) {
                graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));
                initial.update(existing_nodes[dest_node], Pose2(x, y, theta));
            } else {
                existing_nodes.insert(pair<string, unsigned int>(dest_node, ++key));
                graph.add(BetweenFactor<Pose2>(existing_nodes[src_node], existing_nodes[dest_node], Pose2(rel.T.x(), rel.T.y(), rel.R.z()), odom_noise_model));
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

        cout.precision(9);
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

    ofstream os2("coords.txt");
    for (int i = 1; i < graph.size(); ++i) {
        auto factor = graph.at(i);
        auto keys = factor->keys();

        auto src = keys[0];
        auto dest = keys[1];

        Pose2 p1 = result.at<Pose2>(src);
        Pose2 p2 = result.at<Pose2>(dest);

        Pose2 i1 = initial.at<Pose2>(src);
        Pose2 i2 = initial.at<Pose2>(dest);

        os2 << p1.x() << " " << p1.y() << " " << p2.x() << " " << p2.y() << " " << i1.x() << " " << i1.y() << " " << i2.x() << " " << i2.y() << std::endl;
    }

    return 0;
}