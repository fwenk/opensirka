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
#include <OrientationFromMotion/single_sensor_state.h>
#include <MathHelper/expm.h>

BOOST_AUTO_TEST_CASE(trivialboxminus)
{
    SensorState s1, s2;
    Vector9f d = s2 - s1;
    BOOST_CHECK(d.isApprox(Vector9f::Zero()));
}

BOOST_AUTO_TEST_CASE(boxplus)
{
    SensorState s;
    for (int i = 0; i < 1000; ++i) {
        const Vector9f d = Vector9f::Random();
        SensorState s2 = s + d;
        const Vector9f d2 = s2 - s;
        BOOST_CHECK(d.isApprox(d2));
    }
}

BOOST_AUTO_TEST_CASE(boxplus_stacked)
{
    const int numSensors = 15;
    SensorState s1[numSensors];
    SensorState s2[numSensors];
    for (int i = 0; i < 100; ++i) {
        const VectorXf d = VectorXf::Zero(SensorState::DOF * numSensors);
        arraybplus(s2, s1, d, numSensors);
        VectorXf d2(numSensors * SensorState::DOF);
        arraybminus(d2, s2, s1, numSensors);
        BOOST_CHECK(d2.isApprox(d));

        // Check in-place boxplus computes the same result.
        arraybplus(s1, d, numSensors);
        arraybminus(d2, s2, s1, numSensors);
        BOOST_CHECK(d2.isApprox(VectorXf::Zero(SensorState::DOF * numSensors)));
    }
}

BOOST_AUTO_TEST_CASE(jacobians)
{
    for (int k = 0; k < 100; ++k) {
        SensorState s;
        {
            const Vector3f q = Vector3f::Random();
            Vector3f v = Vector3f::Random();
            v *= 20.0f / v.norm();
            Vector3f b = Vector3f::Random();
            b *= 0.5f/180.0f*M_PI / b.norm();
            const Vector9f d = (Vector9f() << q, v, b).finished();
            s += d;
        }
        Accumulate a(Eigen::Quaternionf::UnitRandom(), Vector3f::Random() * 10.0f, 0.8f);

        Matrix9f dummyA;
        Matrix9f6 dummyB;
        Matrix9f A_numeric = Matrix9f::Zero();
        const float epsilon = 1e-3f;
        for (int d = 0; d < 9; ++d) {
            Vector9f delta = Vector9f::Zero();
            delta(d) = epsilon;
            SensorState splus = s + delta;
            SensorState sminus = s + (-delta);
            splus.single_dynamic_model(a, dummyA, dummyB);
            sminus.single_dynamic_model(a, dummyA, dummyB);
            A_numeric.col(d) = (splus - sminus) / (2.0f * epsilon);
        }

        Matrix9f6 B_numeric = Matrix9f6::Zero();
        const auto accbplus = [](const Accumulate& a, const Vector6f& d) -> Accumulate {
            const Eigen::AngleAxisf drot(d.head<3>().norm(), d.head<3>().normalized());
            return Accumulate(a.Q * drot, a.v + d.tail<3>(), a.duration);
        };
        for (int d = 0; d < 6; ++d) {
            Vector6f delta = Vector6f::Zero();
            delta(d) = epsilon;
            const Accumulate aplus = accbplus(a, delta);
            const Accumulate aminus = accbplus(a, -delta);
            SensorState splus(s);
            SensorState sminus(s);
            splus.single_dynamic_model(aplus, dummyA, dummyB);
            sminus.single_dynamic_model(aminus, dummyA, dummyB);
            B_numeric.col(d) = (splus - sminus) / (2.0f * epsilon);
        }

        Matrix9f A;
        Matrix9f6 B;
        s.single_dynamic_model(a, A, B);
        const Matrix9f modelByState_diff = A - A_numeric;
        BOOST_CHECK(modelByState_diff.norm() < 0.1f);
        const Matrix9f6 modelByAcc_diff = B - B_numeric;
        BOOST_CHECK(modelByAcc_diff.norm() < 0.1f);
    }
}
