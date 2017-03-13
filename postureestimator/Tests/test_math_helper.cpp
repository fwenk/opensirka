/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#define BOOST_TEST_MODULE "OrientationFromMotion"

#include <boost/test/unit_test.hpp>
#include <boost/log/trivial.hpp>

#include <Eigen/Dense>
#include <MathHelper/expm.h>
#include <MathHelper/crossx.h>
#include <MathHelper/sphere.h>

static void test_expm(const Eigen::Vector3f& q)
{
    Eigen::Matrix3f Q_expected;
    Eigen::Matrix3f Q;
    const float angle = q.norm();
    if (angle < 1e-10) {
        Q_expected = Eigen::Matrix3f::Identity();
    } else {
        const Eigen::Vector3f axis = q / angle;
        const Eigen::AngleAxisf aa(angle, axis);
        Q_expected = aa.toRotationMatrix();
    }
    Rot::expm(Q, q);
    const Eigen::Matrix3f deltaQ = Q_expected.transpose() * Q;
    BOOST_CHECK(Eigen::Matrix3f::Identity().isApprox(deltaQ));
}

BOOST_AUTO_TEST_CASE(Rot_expm)
{
    test_expm(Eigen::Vector3f::Zero());
    Eigen::Vector3f q = Eigen::Vector3f::Random();
    q.normalize();
    q *= M_PI;
    test_expm(q);
    for (int i = 0; i < 100; ++i)
        test_expm(Eigen::Vector3f::Random());
    for (int i = 0; i < 100; ++i) {
        Eigen::Vector3f q = Eigen::Vector3f::Random();
        q.normalize();
        q *= Eigen::Matrix<float, 1, 1>::Random() * 5.0f/180.0f*M_PI;
        test_expm(q);
    }
}

static void test_logm(const Eigen::Vector3f& q_expected)
{
    Eigen::Matrix3f Q;
    Rot::expm(Q, q_expected);
    Eigen::Vector3f q;
    Rot::logm(q, Q);
    const Eigen::Vector3f q_diff = q - q_expected;
    if (q_diff.norm() < 0.0001f/180.0f*M_PI) {
        BOOST_CHECK(true);
    } else {
        BOOST_LOG_TRIVIAL(warning) << "logm failed for q_expected=" << q_expected << ". q=" << q;
        BOOST_CHECK(false);
    }
}
BOOST_AUTO_TEST_CASE(Rot_logm)
{
    Eigen::Vector3f q = Eigen::Vector3f::Ones();
    Rot::logm(q, Eigen::Matrix3f::Identity());
    BOOST_CHECK(Eigen::Vector3f::Zero().isApprox(q));

    { // Test case that failed with a less accurate implementation.
        Eigen::Vector3f q_expected(0.0000355012744f, 0.00000219319168f, 0.000228781151f);
        test_logm(q_expected);
    }

    for (int i = 0; i < 100; ++i) {
        test_logm(Eigen::Vector3f::Random());
    }
    for (int i = 0; i < 10000; ++i) {
        Eigen::Vector3f q_expected = Eigen::Vector3f::Random();
        q_expected.normalize();
        q_expected *= Eigen::Matrix<float, 1, 1>::Random() * 3.0f/180.0f*M_PI;
        test_logm(q_expected);
    }
}

BOOST_AUTO_TEST_CASE(crossx_float)
{
    for (int i = 0; i < 1000; ++i) {
        const Eigen::Vector3f q = Eigen::Vector3f::Random();
        const Eigen::Vector3f v = Eigen::Vector3f::Random();
        const Eigen::Vector3f qxv_candidate = crossx(q)*v;
        const Eigen::Vector3f qxv_expected = q.cross(v);
        BOOST_CHECK(qxv_expected.isApprox(qxv_candidate));
    }
}

BOOST_AUTO_TEST_CASE(crossx_double)
{
    for (int i = 0; i < 1000; ++i) {
        const Eigen::Vector3d q = Eigen::Vector3d::Random();
        const Eigen::Vector3d v = Eigen::Vector3d::Random();
        const Eigen::Vector3d qxv_candidate = crossx(q)*v;
        const Eigen::Vector3d qxv_expected = q.cross(v);
        BOOST_CHECK(qxv_expected.isApprox(qxv_candidate));
    }
}

BOOST_AUTO_TEST_CASE(unsignedoverflow)
{
    unsigned notwrapped = UINT32_MAX - 1u;
    unsigned wrapped = UINT32_MAX + 2u;
    unsigned diff = notwrapped - wrapped;
    int signedDiff = (int) diff;
    BOOST_CHECK_EQUAL(signedDiff, -1 -2);
}

BOOST_AUTO_TEST_CASE(rotationize_gradient)
{
    for (int i = 0; i < 1000; ++i) { // Does not destroy nearly perfect rotation matrix
        const Eigen::Vector3f sa = Eigen::Vector3f::Random() * 2.0f * M_PI;
        Eigen::Matrix3f Q;
        Rot::expm(Q, sa);
        const Eigen::Matrix3f Q_expected(Q);
        Rot::rotationize_gradient(Q);
        BOOST_CHECK(Q_expected.isApprox(Q));
    }
    for (int i = 0; i < 1000; ++i) { // Does improve the situation on a disturbed rotation matrix
        const Eigen::Vector3f sa = Eigen::Vector3f::Random() * 2.0f * M_PI;
        Eigen::Matrix3f Q;
        Rot::expm(Q, sa);
        // Add disturbance to Q
        Q += Eigen::Matrix3f::Random() * 1e-2f;
        const float disturbed_badness = (Q.transpose() * Q - Eigen::Matrix3f::Identity()).norm();
        Rot::rotationize_gradient(Q);
        const float corrected_badness = (Q.transpose() * Q - Eigen::Matrix3f::Identity()).norm();
        BOOST_CHECK(disturbed_badness > corrected_badness);
    }
}

static Eigen::Matrix<float, 3, 2> numerical_sphere_bplus_jacobian(const Eigen::Vector3f& axis)
{
    Eigen::Matrix<float, 3, 2> J; /*< The jacobian matrix. */
    const float epsilon = 1e-3f; /*< The difference step size. */
    for (int d = 0; d < 2; ++d) {
        Eigen::Vector2f delta;
        delta.setZero();
        delta(d) = epsilon;
        const Eigen::Vector3f aplus  = Sphere::bplus(axis, delta);
        const Eigen::Vector3f aminus = Sphere::bplus(axis, -delta);
        J.col(d) = (aplus - aminus) / (2.0f * epsilon);
    }
    return J;
}

static void test_sphere_bplus_jacobian(const Eigen::Vector3f& axis)
{
    const Eigen::Matrix<float, 3, 2> J_numeric = numerical_sphere_bplus_jacobian(axis);
    const Eigen::Matrix<float, 3, 2> J_analytic = Sphere::dotbplus(axis);
    BOOST_CHECK(J_analytic.isApprox(J_numeric, 1e-3f));
}

BOOST_AUTO_TEST_CASE(sphere_bplus_jacobian)
{
    /* Analytic sphere-bplus-jacobian.
       To do so, it is compared to the numerically computed jacobian.
       First, the 3d-unit axes are tested, then a bunch of randomly generated
       unit length vetors. */
    for (int d = 0; d < 3; ++d)
        test_sphere_bplus_jacobian(Eigen::Matrix3f::Identity().col(d));
    for (int k = 0; k < 100; ++k)
        test_sphere_bplus_jacobian(Eigen::Vector3f::Random().normalized());
}
