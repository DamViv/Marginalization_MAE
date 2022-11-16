#include "file_utils.h"

#include <cstring>
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

void extract_poses(string& file_path, vector<Point>& poses) {
    // Read text file
    ifstream fin;
    fin.open(file_path);

    if (fin.fail()) {
        cout << "can't open the text file." << endl;
        return;
    }

    string line;
    string trash;
    const int LENGTH_SIZE = 128;
    char word[] = "pos=\"";
    char tmp_line[LENGTH_SIZE];
    int word_length = get_length(word);

    while (!fin.eof()) {
        getline(fin, line);

        if (fin.fail() || line == "") {
            fin.clear();
            fin >> trash;
        } else {
            for (int i = 0; i < LENGTH_SIZE; ++i) {
                tmp_line[i] = '\0';
            }
            for (int i = 0; i < line.size(); ++i) {
                tmp_line[i] = line[i];
            }
            tmp_line[LENGTH_SIZE - 1] = '\0';

            int startIdx = index_of(tmp_line, word);
            if (startIdx >= 0) {
                startIdx += word_length;
                char* partial_line = tmp_line + startIdx;
                char* token = tokenize(partial_line, ",");
                float pose_y = stof(token);

                token = tokenize(NULL, "!");
                float pose_x = stof(token);

                poses.push_back(Point(pose_x, pose_y));
                cout << line << endl;
            } else {
                continue;
            }

            // cout << "Relative position (x: " << odometry[2] << ", y: " << odometry[3] << ", theta: " << odometry[4] << ")" << endl;
        }
    }
    fin.close();
}
