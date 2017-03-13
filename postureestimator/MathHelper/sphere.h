/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __SPHERE_H_
#define __SPHERE_H_

#include <Eigen/Dense>

namespace Sphere {
    void bplus_inplace(Eigen::Vector3f& r, const Eigen::Vector2f& delta);
    Eigen::Vector3f bplus(const Eigen::Vector3f& r, const Eigen::Vector2f& delta);
    Eigen::Matrix<float, 3, 2> dotbplus(const Eigen::Vector3f& r); // at delta=0
    Eigen::Matrix3f rotx(const Eigen::Vector3f& r);
};

#endif // __SPHERE_H_
