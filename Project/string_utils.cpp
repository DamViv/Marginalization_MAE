#include "string_utils.h"

std::vector<std::string> split(std::string str, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string temp;

    while (getline(ss, temp, delimiter)) {
        result.push_back(temp);
    }

    return result;
}

std::vector<double> double_parse(std::vector<std::string> split_line) {
    std::vector<double> tmp;
    tmp.reserve(5);
    tmp.push_back(std::stod(split_line[0]));
    tmp.push_back(std::stod(split_line[1]));
    tmp.push_back(std::stod(split_line[2]));
    tmp.push_back(std::stod(split_line[3]));
    tmp.push_back(std::stod(split_line[7]));

    return tmp;
}