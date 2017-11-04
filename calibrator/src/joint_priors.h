/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Authors:
 *   2016: Felix Wenk <felix.wenk@dfki.de>
 *   2017: Felix Wenk <felixwenk@googlemail.com>
 */
#ifndef __JOINT_PRIORS_H_
#define __JOINT_PRIORS_H_

#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>

struct JointConstraintPrior
{
    const Eigen::Vector3d& angular_velocity_pred;
    const Eigen::Vector3d& angular_velocity_succ;
    const double stddev;

    JointConstraintPrior(const Eigen::Vector3d& angular_velocity_pred,
                         const Eigen::Vector3d& angular_velocity_succ,
                         const double stddev)
    : angular_velocity_pred(angular_velocity_pred), angular_velocity_succ(angular_velocity_succ), stddev(stddev) {}

    template<typename T>
    bool operator()(const T *q_pred, const T *v_pred, const T *q_succ, const T *v_succ,
                    const T *r_jointInPredecessor, const T *r_jointInSuccessor,
                    T *residual) const {
        const T omega_pred[3] = {
            T(angular_velocity_pred.x()),
            T(angular_velocity_pred.y()),
            T(angular_velocity_pred.z())
        };
        const T omega_succ[3] = {
            T(angular_velocity_succ.x()),
            T(angular_velocity_succ.y()),
            T(angular_velocity_succ.z())
        };
        T psi_pred[3];
        ceres::CrossProduct(omega_pred, r_jointInPredecessor, psi_pred);
        T psi_pred_world[3];
        ceres::QuaternionRotatePoint(q_pred, psi_pred, psi_pred_world);
        T psi_succ[3];
        ceres::CrossProduct(omega_succ, r_jointInSuccessor, psi_succ);
        T psi_succ_world[3];
        ceres::QuaternionRotatePoint(q_succ, psi_succ, psi_succ_world);

        const T v_joint_pred[3] = {
            v_pred[0] + psi_pred_world[0],
            v_pred[1] + psi_pred_world[1],
            v_pred[2] + psi_pred_world[2],
        };
        const T v_joint_succ[3] = {
            v_succ[0] + psi_succ_world[0],
            v_succ[1] + psi_succ_world[1],
            v_succ[2] + psi_succ_world[2],
        };

        const T stddev_jet(stddev);
        residual[0] = (v_joint_succ[0] - v_joint_pred[0]) / stddev_jet;
        residual[1] = (v_joint_succ[1] - v_joint_pred[1]) / stddev_jet;
        residual[2] = (v_joint_succ[2] - v_joint_pred[2]) / stddev_jet;

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d& angular_velocity_pred,
                                       const Eigen::Vector3d& angular_velocity_succ,
                                       const double stddev) {
        return new ceres::AutoDiffCostFunction<JointConstraintPrior, 3, 4, 3, 4, 3, 3, 3>(
           new JointConstraintPrior(angular_velocity_pred, angular_velocity_succ, stddev));
    }
};

struct VelocityPrior {
    const double stddev;

    VelocityPrior(const double stddev) : stddev(stddev) {}

    template<typename T>
    bool operator()(const T *velocity, T *residual) const {
        const T stddev_jet(stddev);
        residual[0] = velocity[0] / stddev_jet;
        residual[1] = velocity[1] / stddev_jet;
        residual[2] = velocity[2] / stddev_jet;
        return true;
    }

    static ceres::CostFunction *Create(const double stddev) {
        return new ceres::AutoDiffCostFunction<VelocityPrior, 3, 3>(new VelocityPrior(stddev));
    }
};

#endif
