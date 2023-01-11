#pragma
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

void get_markov_blanket(gtsam::KeySet& markov_blanket, gtsam::NonlinearFactorGraph& graph, gtsam::KeySet& nodes_to_remove);