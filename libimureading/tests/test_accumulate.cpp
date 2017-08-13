/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 * Copyright (c) 2017 Felix Wenk
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#define BOOST_TEST_MODULE "Accumulate"

#include "../src/Accumulate.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/test/unit_test.hpp>
//#include <boost/log/trivial.hpp>
#include <boost/bind.hpp>

#include <sstream>
#include <cmath>

namespace bf = boost::filesystem;
using namespace Eigen;
typedef LIR::Accumulate<float> Accumulate;

static void check_accumulate_inverse_consistency(const Accumulate& m01, const Accumulate& m12)
{
    const Accumulate m02 = m01 * m12;
    const Accumulate m12_recovered = m01 % m02;
    BOOST_CHECK(m12.Q.isApprox(m12_recovered.Q));
    BOOST_CHECK_SMALL((m12.v - m12_recovered.v).norm(), 1e-3f);
    BOOST_CHECK_EQUAL(m12.duration, m12_recovered.duration);
}
BOOST_AUTO_TEST_CASE(accumulate_differences)
{
    const Accumulate m01(Quaternionf::Identity(), Vector3f::Ones() * 2.0f, 2u*1000000u);
    const Accumulate m12(Quaternionf::Identity(), Vector3f::Ones(), 1000000u);

    const Accumulate m02 = m01 * m12;
    BOOST_CHECK(m02.Q.isApprox(Quaternionf::Identity()));
    BOOST_CHECK(m02.v.isApprox(Vector3f::Constant(3.0f)));
    BOOST_CHECK_EQUAL(m02.duration, 3*1000000);

    check_accumulate_inverse_consistency(m01, m12);

    for (int i = 0; i < 1000; ++i) {
        const Quaternionf Q01 = Quaternionf::UnitRandom(), Q12 = Quaternionf::UnitRandom();
        Eigen::Matrix<float, 1, 1> length01 = Eigen::Matrix<float, 1, 1>::Random() * 40.0f + Eigen::Matrix<float,1,1>::Constant(10.0f);
        Eigen::Matrix<float, 1, 1> length12 = Eigen::Matrix<float, 1, 1>::Random() * 40.0f + Eigen::Matrix<float,1,1>::Constant(10.0f);
        check_accumulate_inverse_consistency(Accumulate(Q01, Vector3f::Random().normalized() * length01, 1000000u),
                                             Accumulate(Q12, Vector3f::Random().normalized() * length12, 1100000u));
    }
}

BOOST_AUTO_TEST_CASE(accumulate_overflow_trivial)
{
    Accumulate m01(Quaternionf::Identity(), Vector3f(0.0f, 0.0f, -90.0f), 1000000u);
    m01 *= Accumulate(Quaternionf::Identity(), Vector3f::Zero(), 0u);
    BOOST_CHECK_CLOSE(m01.v.z(), 10.0f, 1.0f);
}

BOOST_AUTO_TEST_CASE(accumulate_overflow)
{
    {
        Accumulate m01(Quaternionf::Identity(), Vector3f(0.0f, 0.0f, -40.0f), 1000000u);
        Accumulate m12(Quaternionf::Identity(), Vector3f(0.0f, 0.0f, -1300.0f), 1000000u);
        Accumulate m02 = m01 * m12;
        for (int k = 0; k < 3; ++k)
            BOOST_CHECK_LE(std::abs(m02.v[k]), 50.0f);
    }{
        Accumulate m01(Quaternionf::Identity(), Vector3f(0.0f, 0.0f, -40.0f), 1000000u);
        Accumulate m12(Quaternionf::Identity(), Vector3f(0.0f, 0.0f, -30.0f), 1000000u);
        Accumulate m02 = m01 * m12;
        BOOST_CHECK_CLOSE(m02.v.z(), 30.0f, 1.0f);
        check_accumulate_inverse_consistency(m01, m12);
    }{
        const Vector3f axis01(0.877629637f, 0.321868062f, -0.364222646f);
        const Vector3f axis12(0.510310769f, 0.792763352f, -0.0254300833f);
        Quaternionf Q01; Q01 = AngleAxisf(axis01.norm(), axis01.normalized());
        Quaternionf Q12; Q12 = AngleAxisf(axis12.norm(), axis12.normalized());
        Accumulate m01(Q01, Vector3f(29.8361607f, -43.6177063f, 17.1878986f).normalized() * 49.0f, 1000000u);
        Accumulate m12(Q12, Vector3f(-22.9289055f, 33.9052544f, 45.6504478f).normalized() * 49.0f, 1000000u);
        check_accumulate_inverse_consistency(m01, m12);
    }
}
