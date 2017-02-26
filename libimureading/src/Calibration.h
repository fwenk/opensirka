/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __CALIBRATION_H_
#define __CALIBRATION_H_

#include <Eigen/Dense>

namespace LIR {
    class Calibration {
        Eigen::Matrix3d accelGain_;
        Eigen::Vector3d accelOffset_;
        Eigen::Vector3d gyroOffset_;
    public:
        Calibration(const Eigen::Matrix3d& accelGain, const Eigen::Vector3d& accelOffset,
                    const Eigen::Vector3d& gyroOffset);
        Calibration(const std::string& file);

        Eigen::Matrix3d accelGain() const { return accelGain_; }
        Eigen::Vector3d accelOffset() const { return accelOffset_; }
        Eigen::Vector3d gyroOffset() const { return gyroOffset_; }
        
        void save(const std::string& file);
    };
}

#endif // __CALIBRATION_H_
