#include "file_utils.h"

#include <fstream>
#include <iostream>

#include "string_utils.h"

using namespace std;

void extract_data(string& file_path, vector<vector<double>>& odometries) {
    // Read text file
    ifstream fin;
    fin.open(file_path);

    if (fin.fail()) {
        cout << "can't open the text file." << endl;
        return;
    }

    string line;
    string trash;

    while (!fin.eof()) {
        getline(fin, line);

        if (fin.fail() || line == "") {
            fin.clear();
            fin >> trash;
        } else {
            vector<std::string> split_line = split(line, ' ');
            vector<double> odometry = double_parse(split_line);
            odometries.push_back(odometry);
            // cout << "Relative position (x: " << odometry[2] << ", y: " << odometry[3] << ", theta: " << odometry[4] << ")" << endl;
        }
    }
    fin.close();
}

void write_data(std::string& file_path, std::vector<std::vector<double>>& odometries) {
    // Write the sorted data in text file
    ofstream fout;
    fout.open(file_path, ios_base::in | ios_base::out | ios_base::trunc);
    fout << fixed;

    for (int i = 0; i < odometries.size(); ++i) {
        for (int j = 0; j < odometries[0].size(); ++j) {
            if (j != odometries[0].size() - 1) {
                fout << odometries[i][j] << " ";
            } else {
                fout << odometries[i][j] << '\n';
            }
        }
    }
    fout.close();
}