#ifndef STRING_UTILS_H
#define STRING_UTILS_H

#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> split(std::string str, char deliminater);

std::vector<double> double_parse(std::vector<std::string>);
#endif /* STRING_UTILS_H */