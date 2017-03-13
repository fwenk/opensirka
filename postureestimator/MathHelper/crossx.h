/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __CROSSX_H_
#define __CROSSX_H_

#include <Eigen/Dense>

Eigen::Matrix3f crossx(const Eigen::Vector3f& v);
Eigen::Matrix3d crossx(const Eigen::Vector3d& v);

#endif // __CROSSX_H_
