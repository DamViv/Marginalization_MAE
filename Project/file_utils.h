#pragma once

#include <string>
#include <vector>

#include "point.h"
void extract_data(std::string& file_path, std::vector<std::vector<double>>& odometries);

void write_data(std::string& file_path, std::vector<std::vector<double>>& odometries);

void extract_poses(std::string& file_path, std::vector<Point>& poses);
