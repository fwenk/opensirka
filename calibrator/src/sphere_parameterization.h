/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __SPHERE_PARAMETERIZATION_
#define __SPHERE_PARAMETERIZATION_

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/jet.h>

template<typename T>
void boxplus(const T *x, const T *delta, T *x_bplus_delta)
{
    const T alpha = atan2(x[2], x[1]);
    const T r = sqrt(x[2]*x[2] + x[1]*x[1]);
    const T c = cos(alpha);
    const T s = sin(alpha);
    const T squared_norm_delta = delta[0] * delta[0] + delta[1] * delta[1];

    T exp_delta[3];
    if (squared_norm_delta > T(0.0)) {
        const T norm_delta = sqrt(squared_norm_delta);
        const T cos_norm_delta = cos(norm_delta);
        const T sinc_norm_delta = sin(norm_delta) / norm_delta;
        exp_delta[0] = cos_norm_delta;
        exp_delta[1] = sinc_norm_delta * delta[0];
        exp_delta[2] = sinc_norm_delta * delta[1];
    } else {
        // We do not just use exp_delta = [1,0,0] here because that is a
        // constant and when used for automatic differentiation will
        // lead to a zero derivative. Instead we take a first order
        // approximation and evaluate it at zero.
        exp_delta[0] = T(1.0);
        exp_delta[1] = delta[0];
        exp_delta[2] = delta[1];
    }

    x_bplus_delta[0] = x[0] * exp_delta[0] -        r * exp_delta[1];
    x_bplus_delta[1] = x[1] * exp_delta[0] + x[0] * c * exp_delta[1] - s * exp_delta[2];
    x_bplus_delta[2] = x[2] * exp_delta[0] + x[0] * s * exp_delta[1] + c * exp_delta[2];
}

template<typename T>
void boxminus(const T *y, const T *x, T *y_bminus_x) {
    const T alpha = atan2(x[2], x[1]);
    const T r = sqrt(x[2]*x[2] + x[1]*x[1]);
    const T c = cos(alpha);
    const T s = sin(alpha);

    T Ry[3] = { x[0] * y[0] +     x[1] * y[1] +     x[2] * y[2],
                  -r * y[0] + x[0] * c * y[1] + x[0] * s * y[2],
                                    -s * y[1] +        c * y[2] };

    const T squared_norm_v = Ry[1]*Ry[1] + Ry[2]*Ry[2];
    T norm_v = sqrt(squared_norm_v);
    if (norm_v < T(1e-6)) {
        if (Ry[0] < T(0.0)) {
            using namespace std; // Silence warning about integer abs();
            if (abs(Ry[2]) < abs(Ry[1])) {
                y_bminus_x[1-1] = T(0.0);
                y_bminus_x[2-1] = atan2(abs(Ry[2]), Ry[0]);
            } else {
                y_bminus_x[1-1] = atan2(abs(Ry[1]), Ry[0]);
                y_bminus_x[2-1] = T(0.0);
            }
            return;
        }
        norm_v = T(1e-6);
    }
    const T fac = atan2(norm_v, Ry[0]);
    y_bminus_x[0] = fac * Ry[1];
    y_bminus_x[1] = fac * Ry[2];
}

struct SpherePlus {
    template<typename T>
    bool operator()(const T *x, const T *delta, T *x_plus_delta) const {
        boxplus<T>(x, delta, x_plus_delta);
        return true;
    }
};

class SphereParameterization : public ceres::LocalParameterization
{
    SpherePlus spherePlus;
public:
    virtual int GlobalSize() const { return 3; }
    virtual int LocalSize() const { return 2; }
    virtual bool Plus(const double *x, const double *delta,
                      double * x_plus_delta) const {
        spherePlus(x, delta, x_plus_delta);
        return true;
    }
    virtual bool ComputeJacobian(const double *x, double *jacobian) const {
        // r might be zero. That case is handled by atan2 instead of computing
        // the sin and cos by dividing by r.
        const double r = std::hypot(x[1], x[2]);
        const double alpha = std::atan2(x[2], x[1]);
        const double c = std::cos(alpha);
        const double s = std::sin(alpha);

        *(jacobian++) = -r;
        *(jacobian++) = 0.0;
        *(jacobian++) = x[0] * c;
        *(jacobian++) = -s;
        *(jacobian++) = x[0] * s;
        *(jacobian++) = c;

        return true;
    }
};

#endif // __SPHERE_PARAMETERIZATION_
