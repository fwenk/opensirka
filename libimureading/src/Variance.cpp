/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */

#include "Variance.h"

#include <string>
#include <fstream>

using namespace std;

LIR::NoiseDensities::NoiseDensities(const string& filename)
{
    ifstream ifs(filename);
    if (!ifs.is_open())
        throw "Could not open noise density file.";
    if (ifs.eof())
        throw "Noise density file is empty.";

    string line;
    getline(ifs, line);
    if (line[0] != '#')
        throw "Noise density file must start with a comment.";

    for (int r = 1; r < 3; ++r) {
        if (ifs.eof())
            throw "Unexpected EOF";
        int type = INT_MAX;
        double value;
        getline(ifs, line);
        stringstream ss(line);
        ss >> type;
        ss >> value;
        switch (type) {
            case 1:
                accelNoiseDensity = value;
                break;
            case 2:
                gyroNoiseDensity = value;
                break;
                
            default:
                throw "Unknown sensor type in noise density file.";
                break;
        }
    }
}

LIR::NoiseDensities::NoiseDensities(const double gyroNoiseDensity, const double accelNoiseDensity)
: gyroNoiseDensity(gyroNoiseDensity), accelNoiseDensity(accelNoiseDensity)
{}


Eigen::Matrix3d LIR::NoiseDensities::gyroCovariance(const double deltaT)
{
    return Eigen::Matrix3d::Identity() * (gyroNoiseDensity * gyroNoiseDensity * deltaT);
}

Eigen::Matrix3d LIR::NoiseDensities::accelCovariance(const double deltaT)
{
    return Eigen::Matrix3d::Identity() * (accelNoiseDensity * accelNoiseDensity * deltaT);
}
