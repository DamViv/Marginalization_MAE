#include "graph_utils.h"

void get_markov_blanket(gtsam::KeySet& markov_blanket, gtsam::NonlinearFactorGraph& graph, gtsam::KeySet& nodes_to_remove) {
    gtsam::KeySet child_nodes;

    for (auto node : nodes_to_remove) {
        markov_blanket.insert(node);
        for (int i = 1; i < graph.size(); ++i) {
            auto node_pair = graph.at(i)->keys();
            if (node_pair[0] == node) {
                markov_blanket.insert(node_pair[1]);
                child_nodes.insert(node_pair[1]);
            } else if (node_pair[1] == node) {
                markov_blanket.insert(node_pair[0]);
            }
        }

        for (int i = 1; i < graph.size(); ++i) {
            auto node_pair = graph.at(i)->keys();
            if (child_nodes.find(node_pair[1]) != child_nodes.end()) {
                markov_blanket.insert(node_pair[0]);
            }
        }
    }
}