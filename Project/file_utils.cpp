#include "file_utils.h"

using namespace std;

void loadGraph_from_relations(string relation_file, std::vector<relation>& relations) {
    ifstream fin;
    fin.open(relation_file);
    if (fin.fail()) {
        cout << "can't open the text file." << endl;
        return;
    }
    string line;
    string trash;

    while (!fin.eof()) {
        getline(fin, line);

        string id1, id2;
        double tx, ty, tz, rx, ry, rz;

        cout << line << endl;

        stringstream ss(line);
        ss >> id1 >> id2 >> tx >> ty >> tz >> rx >> ry >> rz;
        if (fin.fail() || line == "") {
            fin.clear();
            fin >> trash;
        } else {
            Eigen::Vector3d T(tx, ty, tz);
            Eigen::Vector3d R(rx, ry, rz);
            relation rel(id1, id2, R, T);
            relations.push_back(rel);
        }
    }
    fin.close();
}