/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Authors:
 *   2016: Felix Wenk <felix.wenk@dfki.de>
 *   2017: Felix Wenk <felixwenk@googlemail.com>
 */
#ifndef __HINGE_MEASUREMENT_H_
#define __HINGE_MEASUREMENT_H_

#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <ceres/rotation.h>
#include <IMUAccumulate.h>

struct HingeConstraint1D
{
    const double stddev;

    HingeConstraint1D(const double stddev) : stddev(stddev) {}

    template<typename T>
    bool operator()(const T *q_predInWorld, const T *q_succInWorld,
                    const T *axis_inPred, const T *axis_inSucc,
                    T *residual) const {
        const T q_worldInSucc[4] = { q_succInWorld[0], -q_succInWorld[1], -q_succInWorld[2], -q_succInWorld[3] };
        T q_predInSucc[4];
        ceres::QuaternionProduct(q_worldInSucc, q_predInWorld, q_predInSucc);
        T predAxis_inSucc[3];
        ceres::QuaternionRotatePoint(q_predInSucc, axis_inPred, predAxis_inSucc);

        residual[0] = ceres::DotProduct(predAxis_inSucc, axis_inSucc);
        residual[0] = acos(residual[0]);
        residual[0] /= T(stddev);

        return true;
    }

    static ceres::CostFunction *Create(const double stddev) {
        return new ceres::AutoDiffCostFunction<HingeConstraint1D, 1, 4, 4, 3, 3>(new HingeConstraint1D(stddev));
    }
};

struct RectangularJointAxisJointConstraint
{
    template<typename T>
    bool operator()(const T *r_jointInSensor, const T *axis_inSensor, T *residual) const {
        residual[0] = ceres::DotProduct(r_jointInSensor, axis_inSensor);
        return true;
    }

    static ceres::CostFunction *Create() {
        return new ceres::AutoDiffCostFunction<RectangularJointAxisJointConstraint, 1, 3, 3>(new RectangularJointAxisJointConstraint());
    }
};

struct HingeError {
    const LIR::IMUAccumulateEntry& entry_succ;
    const LIR::IMUAccumulateEntry& entry_pred;

    HingeError(const LIR::IMUAccumulateEntry& entry_succ, const LIR::IMUAccumulateEntry& entry_pred)
    : entry_succ(entry_succ), entry_pred(entry_pred) {}

    template<typename T>
    bool operator()(const T *axis_inSucc, const T *axis_inPred, T *residual) const {
        /* Two bodies connected over a hinge must have the same angular velocities,
           except for an angular velocity difference along the joint axis. Thus,
           the angular velocity component perpendicular to the joint axis must
           be the same for both bodies. This is only true up to a constant rotation
           matrix, because the orientation of the sensors relative to the bodies are
           unknown. However, rotations do not affect the norms, so the norms of the
           angular velocities perpendicular to the joint axis must be the same
           for both bodies. */

        const T omega_succ[3] = {
            T(entry_succ.angular_velocity.x()),
            T(entry_succ.angular_velocity.y()),
            T(entry_succ.angular_velocity.z())
        };
        const T omega_pred[3] = {
            T(entry_pred.angular_velocity.x()),
            T(entry_pred.angular_velocity.y()),
            T(entry_pred.angular_velocity.z())
        };

        T omega_perp_succ[3];
        ceres::CrossProduct(omega_succ, axis_inSucc, omega_perp_succ);
        T omega_perp_succ_norm2 = ceres::DotProduct(omega_perp_succ, omega_perp_succ);
        T omega_perp_pred[3];
        ceres::CrossProduct(omega_pred, axis_inPred, omega_perp_pred);
        T omega_perp_pred_norm2 = ceres::DotProduct(omega_perp_pred, omega_perp_pred);
        residual[0] = omega_perp_succ_norm2 - omega_perp_pred_norm2;

        return true;
    }

    static ceres::CostFunction *Create(const LIR::IMUAccumulateEntry& entry_succ,
                                       const LIR::IMUAccumulateEntry& entry_pred)
    {
        return new ceres::AutoDiffCostFunction<HingeError, 1, 3, 3>(new HingeError(entry_succ, entry_pred));
    }
};

#endif
