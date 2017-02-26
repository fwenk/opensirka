/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include "Calibration.h"

#include <fstream>
#include <iomanip>

using namespace Eigen;

LIR::Calibration::Calibration(const Matrix3d& accelGain, const Vector3d& accelOffset, const Vector3d& gyroOffset)
: accelGain_(accelGain), accelOffset_(accelOffset), gyroOffset_(gyroOffset)
{}

LIR::Calibration::Calibration(const std::string& file)
{
    std::ifstream ifs(file);
    std::string line;
    std::getline(ifs, line);
    if (line[0] != '#') {
        throw "Malformed calibration file.";
    }

    // Read accelGain matrix
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            ifs >> accelGain_(r, c);

    // Read accelOffset
    for (int r = 0; r < 3; ++r)
        ifs >> accelOffset_(r);

    // Read gyroOffset
    for (int r = 0; r < 3; ++r)
        ifs >> gyroOffset_(r);
}

void LIR::Calibration::save(const std::string& file)
{
    std::ofstream ofs(file, std::ios_base::trunc);
    if (!ofs.is_open())
        throw "Could not open output stream to save calibration.";

    ofs << "# accelGain (row-major) and accelOffset. Apply as: calibrated = accelGain * (raw - accelOffset)." << std::endl;
    ofs << std::fixed << std::setprecision(16);

    // Save accelGain matrix
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (c)
                ofs << ' ';
            ofs << accelGain_(r, c);
        }
        ofs << std::endl;
    }

    // Save accelOffset.
    for (int r = 0; r < 3; ++r) {
        if (r)
            ofs << ' ';
        ofs << accelOffset_(r);
    }
    ofs << std::endl;

    // Save gyroOffset
    for (int r = 0; r < 3; ++r) {
        if (r)
            ofs << ' ';
        ofs << gyroOffset_(r);
    }
    ofs << std::endl;
}
