#include "graph_utils.h"

Graph::Graph(int V) {
    parent = new int[V];

    // i 0 1 2 3 4 5
    // parent[i] 0 1 2 3 4 5
    for (int i = 0; i < V; i++)
        parent[i] = i;

    G.clear();
    T.clear();
}
void Graph::AddWeightedEdge(int u, int v, float w) {
    G.push_back(make_pair(w, edge(u, v)));
}
int Graph::find_set(int i) {
    // If i is the parent of itself
    if (i == parent[i])
        return i;
    else
        // Else if i is not the parent of itself
        // Then i is not the representative of his set,
        // so we recursively call Find on its parent
        return find_set(parent[i]);
}

void Graph::union_set(int u, int v) {
    parent[u] = parent[v];
}
void Graph::kruskal() {
    int i, uRep, vRep;
    sort(G.begin(), G.end(), MI_descend);

    for (i = 0; i < G.size(); i++) {
        uRep = find_set(G[i].second.first);
        vRep = find_set(G[i].second.second);
        if (uRep != vRep) {
            T.push_back(G[i]);  // add to tree
            union_set(uRep, vRep);
        }
    }
}

void Graph::print() {
    std::cout << "Edge :"
              << " Weight" << std::endl;
    for (int i = 0; i < T.size(); i++) {
        std::cout << T[i].second.first << " - " << T[i].second.second << " : " << T[i].first << std::endl;
    }
}

std::pair<int, int> Graph::get_edge(int i) {
    std::pair<int, int> one_edge;
    one_edge.first = T[i].second.first;
    one_edge.second = T[i].second.second;
    return one_edge;
}

int Graph::size() {
    return T.size();
}

void get_markov_blanket(gtsam::KeySet& markov_blanket, gtsam::NonlinearFactorGraph& graph, gtsam::KeySet& nodes_to_remove) {
    // gtsam::KeySet child_nodes;

    for (auto node : nodes_to_remove) {
        markov_blanket.insert(node);
        for (int i = 1; i < graph.size(); ++i) {
            auto node_pair = graph.at(i)->keys();
            if (node_pair[0] == node) {
                markov_blanket.insert(node_pair[1]);
                // child_nodes.insert(node_pair[1]);
            } else if (node_pair[1] == node) {
                markov_blanket.insert(node_pair[0]);
            }
        }

        /*         for (int i = 1; i < graph.size(); ++i) {
                    auto node_pair = graph.at(i)->keys();
                    if (child_nodes.find(node_pair[1]) != child_nodes.end()) {
                        markov_blanket.insert(node_pair[0]);
                    }
                } */
    }
}