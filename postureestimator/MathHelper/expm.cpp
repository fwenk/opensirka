/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "expm.h"
#include "crossx.h"

/** Rodrigues formula. */
static inline void Q_from_axis_cos_sine(Eigen::Matrix3f& Q, const Eigen::Vector3f& unit_axis, const float c, const float s)
{
    const float& x = unit_axis[0];
    const float& y = unit_axis[1];
    const float& z = unit_axis[2];
    const float cv = 1.0f - c;
    const float xyv = x * y * cv;
    const float yzv = y * z * cv;
    const float xzv = x * z * cv;

    float *d = Q.data();
    // Column-major storage
    *(d++) = x * x * cv + c;    // (1,1)
    *(d++) = xyv + z * s;       // (2,1)
    *(d++) = xzv - y * s;       // (3,1)
    *(d++) = xyv - z * s;       // (1,2)
    *(d++) = y * y * cv + c;    // (2,2)
    *(d++) = yzv + x * s;       // (3,2)
    *(d++) = xzv + y * s;       // (1,3)
    *(d++) = yzv - x * s;       // (2,3)
    *(d++) = z * z * cv + c;    // (3,3)
}

/** Rodrigues formula. */
static inline void Q_from_axis_cos_sine(Eigen::Matrix3d& Q, const Eigen::Vector3d& unit_axis, const double c, const double s)
{
    const double& x = unit_axis[0];
    const double& y = unit_axis[1];
    const double& z = unit_axis[2];
    const double cv = 1.0f - c;
    const double xyv = x * y * cv;
    const double yzv = y * z * cv;
    const double xzv = x * z * cv;

    double *d = Q.data();
    // Column-major storage
    *(d++) = x * x * cv + c;    // (1,1)
    *(d++) = xyv + z * s;       // (2,1)
    *(d++) = xzv - y * s;       // (3,1)
    *(d++) = xyv - z * s;       // (1,2)
    *(d++) = y * y * cv + c;    // (2,2)
    *(d++) = yzv + x * s;       // (3,2)
    *(d++) = xzv + y * s;       // (1,3)
    *(d++) = yzv - x * s;       // (2,3)
    *(d++) = z * z * cv + c;    // (3,3)
}

/**
 *  Calculates cross product matrix of vector q and the matrix exponential.
 *  Double precision variant.
 */
void Rot::expm(Eigen::Matrix3d& Q, const Eigen::Vector3d& q)
{
    const double angle = q.norm();
    if (angle < 5.0f/180.0f*M_PI) {
        const Eigen::Matrix3d qx = crossx(q);
        const Eigen::Matrix3d qx2 = qx * qx;
        const double angle2 = angle * angle;
        const double angle4 = angle2 * angle2;
        const double taylor1 = 1.0f - angle2/6.0f + angle4/120.0f;
        const double taylor2 = 0.5f - angle2/24.0f + angle4/720.0f;
        Q = Eigen::Matrix3d::Identity() + qx * taylor1 + qx2 * taylor2;
    } else {
        const Eigen::Vector3d q_unit = q / angle;
        const double c = cos(angle);
        const double s = sqrt(1 - c*c);
        Q_from_axis_cos_sine(Q, q_unit, c, s);
    }
}

void Rot::expm(Eigen::Matrix3f& Q, const Eigen::Vector3f& q)
{
    const float angle = q.norm();
    if (angle < 5.0f/180.0f*M_PI) {
        const Eigen::Matrix3f qx = crossx(q);
        const Eigen::Matrix3f qx2 = qx * qx;
        const float angle2 = angle * angle;
        const float angle4 = angle2 * angle2;
        const float taylor1 = 1.0f - angle2/6.0f + angle4/120.0f;
        const float taylor2 = 0.5f - angle2/24.0f + angle4/720.0f;
        Q = Eigen::Matrix3f::Identity() + qx * taylor1 + qx2 * taylor2;
    } else {
        const Eigen::Vector3f q_unit = q / angle;
        const float c = cosf(angle);
        const float s = sqrtf(1 - c*c);
        Q_from_axis_cos_sine(Q, q_unit, c, s);
    }
}

static void axis_from_eigendecomp(Eigen::Vector3f& q, const Eigen::Matrix3f& Q)
{
    Eigen::EigenSolver<Eigen::Matrix3f> es(Q);
    const Eigen::EigenSolver<Eigen::Matrix3f>::EigenvectorsType evecs = es.eigenvectors();
    const Eigen::EigenSolver<Eigen::Matrix3f>::EigenvalueType evals = es.eigenvalues();
    int axisidx = -1;
    for (int i = 0; i < 3; ++i) {
        if (fabsf(evals(i).imag()) < 1e-6f && fabsf(evals(i).real() - 1.0f) < 1e-6f) {
            axisidx = i;
            break;
        }
    }
    assert(axisidx != -1);
    q = evecs.col(axisidx).real();
}
void Rot::logm(Eigen::Vector3f& q, const Eigen::Matrix3f& Q)
{
    const float c = fmax(-1.0, fmin(1.0, (Q.trace() - 1.0f) / 2.0f));
    const float angle = acosf(c);
    const float s = sqrtf(1.0f - c * c);
    float f;
    if (fabs(angle) < 5.0f/180.0f*M_PI) { // If angle is small, use Taylor expansion of else-branch.
        const float angle2 = angle * angle;
        const float angle4 = angle2 * angle2;
        f = 1.0f/2.0f + angle2 / 12.0f + angle4 * 7.0f/(30.0f * 24.0f);
    } else if (s < 1e-8f) {
        // If the angle is very close to pi, there's no Taylor expansion.
        // To avoid the division by zero, the axis is computed from the eigenvector decomposition.
        axis_from_eigendecomp(q, Q);
        q *= angle;
        return;
    } else {
        f = angle / (2 * s);
    }

    q[0] = Q(2,1) - Q(1,2);
    q[1] = Q(0,2) - Q(2,0);
    q[2] = Q(1,0) - Q(0,1);
    q *= f;
}

void Rot::rotationize_gradient(Eigen::Matrix3f& Q)
{
    Q -= (Q * Q.transpose() * Q - Q) * 4.0f / 10.0f; // Rate 1/10.
}

