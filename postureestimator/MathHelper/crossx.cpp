/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include "crossx.h"

Eigen::Matrix3f crossx(const Eigen::Vector3f& v)
{
    return (Eigen::Matrix3f() <<
            0.0,    -v(2),  v(1),
            v(2),   0.0,    -v(0),
            -v(1),  v(0),   0.0).finished();
}

Eigen::Matrix3d crossx(const Eigen::Vector3d& v)
{
    return (Eigen::Matrix3d() <<
            0.0,    -v(2),  v(1),
            v(2),   0.0,    -v(0),
            -v(1),  v(0),   0.0).finished();
}
