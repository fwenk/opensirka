/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __OFM_TYPES_H_
#define __OFM_TYPES_H_

#include <Eigen/Dense>
#include <Accumulate.h>

typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Matrix4f Matrix4f;
typedef Eigen::Vector4f Vector4f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 9, 1> Vector9f;
typedef Eigen::Matrix<float, 9, 9> Matrix9f;
typedef Eigen::Matrix<float, 9, 6> Matrix9f6;

typedef Eigen::VectorXf VectorXf;
typedef Eigen::MatrixXf MatrixXf;

typedef LIR::Accumulate<float> Accumulate;

struct Variances {
    float acceleration;
    float acceleration_density;
    float angular_velocity;
    float angular_velocity_density;
    float gyro_bias;
    float joint_velocity_difference_variance;
    float joint_velocity_difference_decorrelation_time;
    float velocity_variance;
    float velocity_decorrelation_time;
    float z_variance;
    float z_decorrelation_time;
    float hinge_axis_variance;
    float hinge_axis_decorrelation_time;
};

/**
 * Structure to store the covariance of a hinge axis.
 * The covariance is stored for the 3d-unit-vector representation of the axis,
 * i.e. as a rank-deficient 3x3-matrix, and not as a 2x2-matrix.
 * The covariance is stored twice: Once in cooridnates of the sensor on the body
 * preceeding the hinge and once in the coordinates of the sensor on the body
 * succeeding the hinge.
 */
struct HingeAxixCovariances {
    Matrix3f covInPredecessor;
    Matrix3f covInSuccessor;
};

#define gravityf (-9.81f)

#endif // __OFM_TYPES_H_
