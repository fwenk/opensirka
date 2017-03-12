/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __JACOBIAN_H_
#define __JACOBIAN_H_

#include <Eigen/Dense>

template <typename ReturnTypeBoxOps, typename ParameterBoxOps,
          typename Function, typename Parameter>
void compute_jacobian(Function f, const Parameter& p,
        Eigen::Matrix<float, ReturnTypeBoxOps::DOF, ParameterBoxOps::DOF>& J)
{
    J.setZero();

    const auto acenter = f(p);
    const float epsilon = 1e-3f;
    for (unsigned d = 0; d < ParameterBoxOps::DOF; ++d) {
        Eigen::Matrix<float, ParameterBoxOps::DOF, 1> delta;
        delta.setZero();
        delta(d) = epsilon;
        const auto aplus = f(ParameterBoxOps::plus(p, delta));
        const auto aminus = f(ParameterBoxOps::plus(p, -delta));
        const auto deltaplus = ReturnTypeBoxOps::minus(aplus, acenter);
        const auto deltaminus = ReturnTypeBoxOps::minus(aminus, acenter);
        J.col(d) = (deltaplus - deltaminus) / (2.0f * epsilon);
    }
}

#endif // __JACOBIAN_H_
