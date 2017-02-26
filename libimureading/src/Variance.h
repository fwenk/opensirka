/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __VARIANCE_H_
#define __VARIANCE_H_

#include <Eigen/Dense>

namespace LIR {
    /**
     * Noise densities are standard deviation per square root time: sigma/sqrt(dt)
     */
    struct NoiseDensities
    {
        double gyroNoiseDensity;
        double accelNoiseDensity;

        NoiseDensities(const std::string& filename);
        NoiseDensities(const double gyroNoiseDensity, const double accelNoiseDensity);

        Eigen::Matrix3d gyroCovariance(const double deltaT);
        Eigen::Matrix3d accelCovariance(const double deltaT);
    };
}

#endif // __VARIANCE_H_
