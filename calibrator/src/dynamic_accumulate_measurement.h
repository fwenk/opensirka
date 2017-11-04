/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Authors:
 *   2016: Felix Wenk <felix.wenk@dfki.de>
 *   2017: Felix Wenk <felixwenk@googlemail.com>
 */
#ifndef __DYNAMIC_ACCUMULATE_MEASUREMENT_H_
#define __DYNAMIC_ACCUMULATE_MEASUREMENT_H_

#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <ceres/rotation.h>

#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <lapacke/lapacke.h>
#endif

#include "packed_triangular_matrix.h"
#include <Accumulate.h>
typedef LIR::Accumulate<double> Accumulate;

struct DynamicAccumulateMeasurement {
    const Accumulate& accumulateDelta;
    const PackedTriangularMatrix<6> inverse_of_cov_sqrt;

    DynamicAccumulateMeasurement(const Accumulate& accumulateDelta,
                                 const PackedTriangularMatrix<6>& inverse_of_cov_sqrt)
    : accumulateDelta(accumulateDelta), inverse_of_cov_sqrt(inverse_of_cov_sqrt) {}

    template<typename T>
    bool operator()(const T *q_pastState, const T *v_pastState, const T *q_futureState, const T *v_futureState, T *residual) const {
        T pastState_Q_worldInImu[4] = { q_pastState[0], -q_pastState[1], -q_pastState[2], -q_pastState[3] };
        T Q_futureInPast[4];
        ceres::QuaternionProduct(pastState_Q_worldInImu, q_futureState, Q_futureInPast);
        T accumulateDelta_Q_inv[4] = { T(accumulateDelta.Q.w()), T(-accumulateDelta.Q.x()), T(-accumulateDelta.Q.y()), T(-accumulateDelta.Q.z()) };
        T deltaQ_quat[4];
        ceres::QuaternionProduct(accumulateDelta_Q_inv, Q_futureInPast, deltaQ_quat);
        T residual_notnormalized[6];
        ceres::QuaternionToAngleAxis(deltaQ_quat, residual_notnormalized);

        const T v_futureInPast_inWorld[3] = {
            v_futureState[0] - v_pastState[0],
            v_futureState[1] - v_pastState[1],
            v_futureState[2] - v_pastState[2] + T(9.81*accumulateDelta.durationSeconds())
        };
        ceres::QuaternionRotatePoint(pastState_Q_worldInImu, v_futureInPast_inWorld, residual_notnormalized+3);
        residual_notnormalized[3] -= T(accumulateDelta.v.x());
        residual_notnormalized[4] -= T(accumulateDelta.v.y());
        residual_notnormalized[5] -= T(accumulateDelta.v.z());
        lower_triangular_packed_matrix_mult<T, 6>(inverse_of_cov_sqrt.elems, residual_notnormalized, residual);

        return true;
    }

    static ceres::CostFunction *Create(const Accumulate& accumulateDelta,
                                       const Eigen::Matrix<double, 6, 6>& covariance) {
        PackedTriangularMatrix<6> inv_of_cov_sqrt;
        inv_of_cov_sqrt.setFromEigenMatrix(covariance);
        char uplo = 'L';
        int n = 6;
        int info = 0;
        dpptrf_(&uplo, &n, inv_of_cov_sqrt.elems, &info);
        if (info != 0) {
            std::cerr << "Could not compute cholesky decomposition of accumulate covariance matrix." << std::endl;
            exit(1);
        }
        char diag = 'N';
        dtptri_(&uplo, &diag, &n, inv_of_cov_sqrt.elems, &info);
        if (info != 0) {
            std::cerr << "Could not compute inverse of lower-triangular Cholesky matrix of accumulate covariance." << std::endl;
            exit(1);
        }
        return new ceres::AutoDiffCostFunction<DynamicAccumulateMeasurement, 6, 4, 3, 4, 3>(new DynamicAccumulateMeasurement(accumulateDelta, inv_of_cov_sqrt));
    }
};


#endif
