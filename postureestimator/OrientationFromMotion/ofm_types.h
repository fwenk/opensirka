/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __OFM_TYPES_H_
#define __OFM_TYPES_H_

#include <Eigen/Dense>
#include <MathHelper/expm.h>
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

struct Accumulate
{
    enum { DOF = 6 };
    Matrix3f Q; /**< Accumulated orientation. (end frame in start frame) */
    Vector3f v; /**< Accumulated velocity, not gravity-compenstated, in start frame. */
    unsigned duration; /**< Accumulation duration in microseconds. */

    Accumulate(const Matrix3f& Q, const Vector3f& v, const float duration) : Q(Q), v(v), duration(static_cast<unsigned int>(duration * 1e6f)) {}
    Accumulate(const Matrix3f& Q, const Vector3f& v, const unsigned int duration) : Q(Q), v(v), duration(duration) {}
    Accumulate(const Accumulate& other) : Q(other.Q), v(other.v), duration(other.duration) {}
    Accumulate() : Q(Matrix3f::Identity()), v(Vector3f::Zero()), duration(0) {}

    Accumulate& operator*=(const Accumulate& rhs)
    {
        v += Q * rhs.v;
        overflow_inplace(v);
        Q *= rhs.Q;
        Rot::rotationize_gradient(Q);
        duration += rhs.duration;
        return *this;
    }
    Accumulate operator*(const Accumulate& rhs) const
    {
        Accumulate r(*this);
        r *= rhs;
        return r;
    }
    Accumulate operator%(const Accumulate& rhs) const
    {
        return Accumulate(Q.transpose() * rhs.Q, Q.transpose() * overflow(rhs.v - v), rhs.duration - duration);
    }
    Accumulate& operator=(const Accumulate& rhs)
    {
        Q = rhs.Q;
        v = rhs.v;
        duration = rhs.duration;
        return *this;
    }
    float durationSeconds() const { return duration * 1e-6f; }
    static void overflow_inplace(Vector3f& v)
    {
        const float periodicity = 50.0f;
        for (int d = 0; d < 3; ++d) {
            v(d) = std::fmod(v(d), 2.0f * periodicity);
            if (v(d) >= periodicity)
                v(d) -= 2.0f * periodicity;
            else if (v(d) < -periodicity)
                v(d) += 2.0f * periodicity;
        }
    }
    static Vector3f overflow(const Vector3f& v)
    {
        Vector3f ov(v);
        overflow_inplace(ov);
        return ov;
    }
};

#define gravityf (-9.81f)

#endif // __OFM_TYPES_H_
