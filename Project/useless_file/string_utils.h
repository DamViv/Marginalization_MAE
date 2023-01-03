#pragma once

#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> split(std::string str, char deliminater);

std::vector<double> double_parse(std::vector<std::string>);

size_t get_length(char* str);

int index_of(const char* str, const char* word);

char* tokenize(char* str_or_null, const char* delims);
