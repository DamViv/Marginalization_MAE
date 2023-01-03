#pragma once

#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "relation.h"

void loadGraph_from_relations(std::string relation_file, std::vector<relation>& relations);