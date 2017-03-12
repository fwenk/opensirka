/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __EXPM_H_
#define __EXPM_H_

#include <Eigen/Dense>

namespace Rot {
    void expm(Eigen::Matrix3f& Q, const Eigen::Vector3f& q);
    void logm(Eigen::Vector3f& q, const Eigen::Matrix3f& Q);

    void rotationize_gradient(Eigen::Matrix3f& Q);
}

#endif // __EXPM_H_
