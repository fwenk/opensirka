/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include "IMUReading.h"

#include <fstream>

using namespace std;
using namespace Eigen;

void LIR::save_resolutions(const std::string &filename, const struct Resolutions &resolutions)
{
    ofstream ofs(filename);
    if (!ofs.is_open())
        throw "Coudl not open file to write resolutions into.";

    ofs << "# 1: Accelerometer; 2: Gyroscope. Apply: raw * resolution = physical value." << endl;
    ofs << "1 " << resolutions.accelerometer << endl;
    ofs << "2 " << resolutions.gyroscope << endl;

    ofs.close();
}

struct LIR::Resolutions LIR::load_resolutions(const string &file)
{
    ifstream ifs(file);
    if (!ifs.is_open())
        throw "Could not open resolutions file.";
    if (ifs.eof())
        throw "The resolutions file is empty.";
    string line;
    getline(ifs, line);
    if (line[0] != '#')
        throw "First line must be the application comment.";

    struct Resolutions resolutions;
    for (int r = 1; r < 3; ++r) { // Resolutions file must have 3 lines
        if (ifs.eof())
            throw "Unexpected EOF in resolutions file.";
        int type = INT_MAX;
        float value;
        getline(ifs, line);
        stringstream ss(line);
        ss >> type;
        ss >> value;
        switch (type) {
            case 1: // Accelerometer type
                resolutions.accelerometer = value;
                break;
            case 2: // Gyrometer type
                resolutions.gyroscope = value;
                break;
            default:
                throw "Unknown sensor type.";
        }
    }

    return resolutions;
}

LIR::IMUReading::IMUReading(const string& file)
{
    ifstream ifs(file);
    if (!ifs.is_open())
        throw "Could not open IMU data input file.";

    long lastframe = -1;
    while (!ifs.eof()) {
        string line;
        getline(ifs, line);
        stringstream ss(line);
        int sensor;
        long frame;
        double x, y, z;
        ss >> sensor;
        ss >> frame;
        ss >> x; ss >> y; ss >> z;
        if (frame != lastframe) {
            reading.push_back(IMUReadingEntry());
            reading.back().frame = frame;
            reading.back().time = static_cast<double>(frame) / 1000000; // Frame is actually a µs time.
        }
        if (frame < lastframe) {
            throw "Frame counter wrapped around.";
        }
        Vector3d value;
        value << x, y, z;
        switch (sensor) {
            case 1: // Accelerometer
                reading.back().accel = value;
                break;
            case 2: // Gyroscope
                reading.back().gyro = value;
                break;
            case 3: // Magnetometer (ignore)
                break;
            default:
                throw "Unknown sensor type.";
        }

        lastframe = frame;
    }
}

LIR::IMUReading::IMUReading(const string& file, const struct Resolutions& resolutions)
{
    ifstream ifs(file);
    if (!ifs.is_open())
        throw "Could not open IMU data input file.";

    long lastframe = -1;
    while (!ifs.eof()) {
        string line;
        getline(ifs, line);
        stringstream ss(line);
        int sensor;
        long frame;
        short x, y, z;
        ss >> sensor;
        ss >> frame;
        ss >> x; ss >> y; ss >> z;
        if (frame != lastframe) {
            reading.push_back(IMUReadingEntry());
            reading.back().frame = frame;
            reading.back().time = static_cast<double>(frame) / 1000000; // Frame is actually a µs time.
        }
        if (frame < lastframe) {
            throw "Frame counter wrapped around.";
        }
        switch (sensor) {
            case 1: // Accelerometer
                reading.back().accel << x * resolutions.accelerometer, y * resolutions.accelerometer, z * resolutions.accelerometer;
                break;
            case 2: // Gyroscope
                reading.back().gyro << x * resolutions.gyroscope, y * resolutions.gyroscope, z * resolutions.gyroscope;
                break;
            case 3: // Magnetometer (ignore)
                break;
            default:
                throw "Unknown sensor type.";
        }

        lastframe = frame;
    }
}
