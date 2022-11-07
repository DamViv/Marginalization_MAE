#pragma once

#include <string>
#include <vector>

void extract_data(std::string& file_path, std::vector<std::vector<double>>& odometries);

void write_data(std::string& file_path, std::vector<std::vector<double>>& odometries);