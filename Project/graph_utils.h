#pragma
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include "algorithm_utils.h"

#define edge std::pair<int, int>

class Graph {
   public:
    Graph(int V);
    void AddWeightedEdge(int u, int v, float w);
    int find_set(int i);
    void union_set(int u, int v);
    void kruskal();
    void print();
    std::pair<int, int> get_edge(int i);
    int size();

   private:
    std::vector<std::pair<float, edge> > G;  // graph
    std::vector<std::pair<float, edge> > T;  // mst
    int* parent;
    int V;  // number of vertices/nodes in graph
};

void get_markov_blanket(gtsam::KeySet& markov_blanket, gtsam::NonlinearFactorGraph& graph, gtsam::KeySet& nodes_to_remove);