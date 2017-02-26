/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include "IMUAccumulate.h"

#include <fstream>

using namespace std;
using namespace Eigen;

LIR::IMUAccumulates::IMUAccumulates(const string& filename)
{
    ifstream ifs(filename);
    if (!ifs.is_open())
        throw "Could not open IMU accumulates file.";

    entries.clear();
    while (!ifs.eof()) {
        string line;
        getline(ifs, line);
        if (line.empty())
            break;
        if (line[0] == '#')
            continue; // Skip comment line.

        stringstream ss(line);
        IMUAccumulateEntry e;
        ss >> e.usecs;
        ss >> e.orientation.x();
        ss >> e.orientation.y();
        ss >> e.orientation.z();
        ss >> e.orientation.w();
        ss >> e.velocity.x();
        ss >> e.velocity.y();
        ss >> e.velocity.z();
        ss >> e.angular_velocity.x();
        ss >> e.angular_velocity.y();
        ss >> e.angular_velocity.z();
        entries.push_back(e);
    }
}
