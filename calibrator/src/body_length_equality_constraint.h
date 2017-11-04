/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Authors:
 *   2016: Felix Wenk <felix.wenk@dfki.de>
 *   2017: Felix Wenk <felixwenk@googlemail.com>
 */
#ifndef __BODY_LENGTH_EQUALITY_CONSTRAINT_H_
#define __BODY_LENGTH_EQUALITY_CONSTRAINT_H_

#include <ceres/ceres.h>
#include <ceres/jet.h>

struct BodyLengthEqualityConstraint
{
    double distanceSquaredStddev;
    BodyLengthEqualityConstraint(const double distanceStddev)
    : distanceSquaredStddev(sqrt(2) * distanceStddev * distanceStddev)
    {}

    template <typename T>
    bool operator()(const T *r_predInSensorA, const T *r_succInSensorA,
                    const T *r_predInSensorB, const T *r_succInSensorB,
                    T *residual) const {
        T deltaA[3];
        T deltaB[3];
        for (unsigned d = 0; d < 3; ++d) {
            deltaA[d] = r_succInSensorA[d] - r_predInSensorA[d];
            deltaB[d] = r_succInSensorB[d] - r_predInSensorB[d];
        }
        const T distanceASq = ceres::DotProduct(deltaA, deltaA);
        const T distanceBSq = ceres::DotProduct(deltaB, deltaB);
        residual[0] = (distanceBSq - distanceASq) / T(distanceSquaredStddev);
        return true;
    }

    static ceres::CostFunction *Create(double distanceStddev) {
        return new ceres::AutoDiffCostFunction<BodyLengthEqualityConstraint, 1, 3, 3, 3, 3>(new BodyLengthEqualityConstraint(distanceStddev));
    }
};

#endif
