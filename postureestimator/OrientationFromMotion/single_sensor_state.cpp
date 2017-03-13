/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include "single_sensor_state.h"

#include <MathHelper/expm.h>
#include <MathHelper/crossx.h>

SensorState::SensorState()
: Q(Matrix3f::Identity()), v(Vector3f::Zero()), b(Vector3f::Zero())
{}

SensorState::SensorState(const Matrix3f& Q, const Vector3f& v, const Vector3f& b)
: Q(Q), v(v), b(b)
{}

SensorState& SensorState::operator+=(const Vector9f &d)
{
    Matrix3f deltaQ;
    Rot::expm(deltaQ, d.head<3>());
    Q *= deltaQ;
    Rot::rotationize_gradient(Q);
    v += d.segment<3>(3);
    b += d.tail<3>();

    return *this;
}

SensorState SensorState::operator+(const Vector9f &d) const
{
    SensorState s(*this);
    s += d;
    return s;
}

Vector9f SensorState::operator-(const SensorState &other) const
{
    Vector3f q;
    Rot::logm(q, other.Q.transpose() * Q);
    return (Vector9f() << q,
                          v - other.v,
                          b - other.b).finished();
}

void arraybplus(SensorState * const s, const VectorXf& d, const int N)
{
    for (int i = 0; i < N; ++i)
        s[i] += d.segment<SensorState::DOF>(i * SensorState::DOF);
}

void arraybplus(SensorState * const s2, const SensorState * const s1, const VectorXf& d, const int N)
{
    for (int i = 0; i < N; ++i)
        s2[i] = s1[i] + d.segment<SensorState::DOF>(i * SensorState::DOF);
}

void arraybminus(VectorXf& d, const SensorState * const s2, const SensorState * const s1, const int N)
{
    for (int i = 0; i < N; ++i)
        d.segment<SensorState::DOF>(i * SensorState::DOF) = s2[i] - s1[i];
}


void SensorState::single_dynamic_model(const Accumulate &a, Matrix9f& jacobianA, Matrix9f6& jacobianB)
{
    const float a_duration_sec = a.durationSeconds();

    const Vector3f deltaV = a.v - a_duration_sec / 2.0f * b.cross(a.v);

    Matrix3f rot_bias;
    Rot::expm(rot_bias, -b * a_duration_sec);
    const Matrix3f deltaQ = (a.Q * rot_bias);

    jacobianA = Matrix9f::Zero(); // TODO: Remove this.
    jacobianA.topLeftCorner<3, 3>() = deltaQ.transpose();
    jacobianA.block<3,3>(3, 0) = -Q * crossx(deltaV);
    jacobianA.block<3,3>(3, 3) = Matrix3f::Identity();
    jacobianA.block<3,3>(0, 6) = -Matrix3f::Identity() * a_duration_sec;
    jacobianA.block<3,3>(3, 6) = Q * a_duration_sec / 2.0f * crossx(a.v);
    jacobianA.block<3,3>(6, 6) = Matrix3f::Identity();

    jacobianB = Matrix9f6::Zero(); // TODO: Remove this.
    jacobianB.topLeftCorner<3, 3>() = rot_bias.transpose();
    jacobianB.block<3, 3>(3, 3) = Q * (Matrix3f::Identity() - crossx(b) * a_duration_sec / 2.0f);

    v += Q * deltaV;
    v(2) += gravityf * a_duration_sec;
    Q *= deltaQ;
}

std::ostream& operator<<(std::ostream& os, const SensorState& s)
{
    os << "Q:" << std::endl << s.Q << std::endl << "v:" << std::endl << s.v << std::endl
        << "b:" << std::endl << s.b;
    return os;
}
