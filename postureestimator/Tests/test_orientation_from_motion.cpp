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

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/test/unit_test.hpp>
//#include <boost/log/trivial.hpp>
#include <boost/bind.hpp>
#include <OrientationFromMotion/orientation_from_motion.h>
#include <OrientationFromMotion/joint_sensor_map.h>
#include <MathHelper/expm.h>
#include <MathHelper/jacobian.h>

#include <sstream>
#include <cmath>

namespace bf = boost::filesystem;

BOOST_AUTO_TEST_CASE(dynamicmodel_jacobians_trivial)
{
    struct Variances variances;
    variances.acceleration = 1.0f;
    variances.acceleration_density = 1.0f;
    variances.angular_velocity = 1.0f;
    variances.angular_velocity_density = 1.0f;
    variances.joint_velocity_difference_variance = 1.0f;
    variances.joint_velocity_difference_decorrelation_time = 1.0f;
    variances.velocity_variance = 1.0f;
    variances.velocity_decorrelation_time = 1.0f;
    variances.z_variance = 1.0f;
    variances.z_decorrelation_time = 1.0f;
    OrientationFromMotion ofm(1, JointSensorMap(0), variances);
    {
        const Vector3f q = Vector3f::Random();
        Vector3f v = Vector3f::Random();
        v *= 20.0f / v.norm();
        Vector3f b = Vector3f::Random();
        b *= 0.5f/180.0f*M_PI / b.norm();
        const Vector9f d = (Vector9f() << q, v, b).finished();
        ofm.sensors[0] += d;
    }
    Accumulate a(Eigen::Quaternionf::UnitRandom(), Vector3f::Random() * 10.0f, 0.8f);
    Accumulate as[1] = { a };

    MatrixXf jacobianA = MatrixXf::Zero(SensorState::DOF, SensorState::DOF);
    MatrixXf jacobianB = MatrixXf::Zero(SensorState::DOF, Accumulate::DOF);

    SensorState s = ofm.sensors[0];
    ofm.dynamic_model(ofm.sensors, jacobianA, jacobianB, as);

    Matrix9f jacobianA_expected;
    Matrix9f6 jacobianB_expected;
    s.single_dynamic_model(a, jacobianA_expected, jacobianB_expected);

    const Vector9f delta = s - ofm.sensors[0];
    BOOST_CHECK(delta.isApprox(Vector9f::Zero()));
    BOOST_CHECK(jacobianA_expected.isApprox(jacobianA));
    BOOST_CHECK(jacobianB_expected.isApprox(jacobianB));
}

BOOST_AUTO_TEST_CASE(dynamicmodel_jacobians)
{
    struct Variances variances;
    variances.acceleration = 1.0f;
    variances.acceleration_density = 1.0f;
    variances.angular_velocity = 1.0f;
    variances.angular_velocity_density = 1.0f;
    variances.joint_velocity_difference_variance = 1.0f;
    variances.joint_velocity_difference_decorrelation_time = 1.0f;
    variances.velocity_variance = 1.0f;
    variances.velocity_decorrelation_time = 1.0f;
    variances.z_variance = 1.0f;
    variances.z_decorrelation_time = 1.0f;
    const int numSensors = 15;
    OrientationFromMotion ofm(numSensors, JointSensorMap(0), variances);
    Accumulate as[numSensors];
    for (int i = 0; i < numSensors; ++i) {
        const Vector3f q = Vector3f::Random();
        Vector3f v = Vector3f::Random();
        v *= 20.0f / v.norm();
        Vector3f b = Vector3f::Random();
        b *= 0.5f/180.0f*M_PI / b.norm();
        const Vector9f d = (Vector9f() << q, v, b).finished();
        ofm.sensors[i] += d;

        as[i].Q = Eigen::Quaternionf::UnitRandom();
        as[i].v = Vector3f::Random() * 10.0f;
        as[i].duration = static_cast<unsigned int>(0.8f * 1000000);
    }

    MatrixXf dummyA(numSensors * SensorState::DOF, numSensors * SensorState::DOF);
    MatrixXf dummyB(numSensors * SensorState::DOF, numSensors * Accumulate::DOF);
    MatrixXf jacobianA_numeric(numSensors * SensorState::DOF, numSensors * SensorState::DOF);
    MatrixXf jacobianB_numeric(numSensors * SensorState::DOF, numSensors * Accumulate::DOF);

    SensorState splus[numSensors];
    SensorState sminus[numSensors];
    const float epsilon = 1e-3f;
    for (int d = 0; d < SensorState::DOF * numSensors; ++d) {
        VectorXf delta = VectorXf::Zero(SensorState::DOF * numSensors);
        delta(d) = epsilon;
        arraybplus(splus, ofm.sensors, delta, numSensors);
        arraybplus(sminus, ofm.sensors, -delta, numSensors);
        ofm.dynamic_model(splus, dummyA, dummyB, as);
        ofm.dynamic_model(sminus, dummyA, dummyB, as);
        VectorXf col(numSensors * SensorState::DOF);
        arraybminus(col, splus, sminus, numSensors);
        jacobianA_numeric.col(d) = col / (2.0f * epsilon);
    }

    const auto abplus = [&numSensors](Accumulate * const as2, const Accumulate * const as1, const VectorXf& d) {
        for (int i = 0; i < numSensors; ++i) {
            Accumulate& a2 = as2[i];
            const Accumulate& a1 = as1[i];
            const Vector6f dblock = d.segment<Accumulate::DOF>(Accumulate::DOF * i);
            Matrix3f drot;
            Rot::expm(drot, dblock.head<3>());
            a2.Q = a1.Q * drot;
            a2.v = a1.v + dblock.tail<3>();
            a2.duration = a1.duration;
        }
    };
    for (int d = 0; d < Accumulate::DOF * numSensors; ++d) {
        VectorXf delta = VectorXf::Zero(Accumulate::DOF * numSensors);
        delta(d) = epsilon;
        Accumulate aplus[numSensors];
        Accumulate aminus[numSensors];
        abplus(aplus, as, delta);
        abplus(aminus, as, -delta);
        for (int i = 0; i < numSensors; ++i) {
            splus[i] = ofm.sensors[i];
            sminus[i] = ofm.sensors[i];
        }
        ofm.dynamic_model(splus, dummyA, dummyB, aplus);
        ofm.dynamic_model(sminus, dummyA, dummyB, aminus);
        VectorXf col(SensorState::DOF * numSensors);
        arraybminus(col, splus, sminus, numSensors);
        jacobianB_numeric.col(d) = col / (2.0f * epsilon);
    }

    MatrixXf jacobianA(numSensors * SensorState::DOF, numSensors * SensorState::DOF);
    MatrixXf jacobianB(numSensors * SensorState::DOF, numSensors * Accumulate::DOF);
    ofm.dynamic_model(ofm.sensors, jacobianA, jacobianB, as);
    MatrixXf jacobianA_diff = jacobianA - jacobianA_numeric;
    BOOST_CHECK(jacobianA_diff.norm() < 0.1);
    MatrixXf jacobianB_diff = jacobianB - jacobianB_numeric;
    BOOST_CHECK(jacobianB_diff.norm() < 0.1);
}

BOOST_AUTO_TEST_CASE(jointsensormap_construction)
{
    JointSensorMap jsm(1);
    jsm.sensors[0].push_back(SensorLocation(Vector3f(1.0f, 0.0f, 0.0f), 0));
    jsm.sensors[0].push_back(SensorLocation(Vector3f(0.0f, 0.0f, 1.0f), 1));
}

BOOST_AUTO_TEST_CASE(jointsensormap_loading)
{
    { // Empty joint sensors map.
        std::stringstream ss;
        ss << "0 joints";
        JointSensorMap jsm(ss);
        BOOST_CHECK(jsm.numJoints == 0);
    }
    { // Joint sensor map
        std::stringstream ss;
        ss << "2 joints" << std::endl
           << "0 0 1.0 1.2 3.2 H 0.0 1.0 0.0" << std::endl
           << "0 1 1.0 1.0 1.0 H 0.0 1.0 0.0" << std::endl
           << "1 2 2.0 2.0 2.0 S 0.0 0.0 0.0" << std::endl
           << "1 1 -1.0 -1.2 1.0 S 0.0 0.0 0.0" << std::endl;
        JointSensorMap jsm(ss);
        BOOST_CHECK(jsm.numJoints == 2);
        BOOST_CHECK(jsm.hinges.size() == 1);

        Vector3f exepcted[2][2] = {{Vector3f(1.0f, 1.2f, 3.2f), Vector3f(1.0f, 1.0f, 1.0f)},
            {Vector3f(-1.0f, -1.2f, 1.0f), Vector3f(2.0f, 2.0f, 2.0f)}};

        BOOST_CHECK(jsm.sensors[0].size() == 2);
        BOOST_CHECK(jsm.sensors[1].size() == 2);

        for (int i = 0; i < 2; ++i) {
            for (const SensorLocation& sl : jsm.sensors[i]) {
                const int j = sl.sensorId - i;
                BOOST_CHECK(j < 2);
                BOOST_CHECK(sl.jointInSensor.isApprox(exepcted[i][j]));
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(measurementmodel_jacobians)
{
    // Create a very simple joint sensor map
    std::stringstream ss;
    ss << "1 joints" << std::endl
       << "0 0 1.0 0.0 0.0 H 0.0 1.0 0.0" << std::endl
       << "0 1 -1.0 0.0 0.0 H 0.0 1.0 0.0" << std::endl /*
                                        << "1 1 1.0 0.0 0.0" << std::endl
                                        << "1 2 -1.0 0.0 0.0" << std::endl */;
    JointSensorMap jsm(ss);
    BOOST_CHECK(jsm.numJoints == 1);
    BOOST_CHECK(jsm.hinges.size() == 1);

    struct Variances variances;
    variances.acceleration = 1.0f;
    variances.acceleration_density = 1.0f;
    variances.angular_velocity = 1.0f;
    variances.angular_velocity_density = 1.0f;
    variances.joint_velocity_difference_variance = 1.0f;
    variances.joint_velocity_difference_decorrelation_time = 1.0f;
    variances.velocity_variance = 1.0f;
    variances.velocity_decorrelation_time = 1.0f;
    variances.z_variance = 1.0f;
    variances.z_decorrelation_time = 1.0f;
    variances.hinge_axis_variance = 0.125f*0.125f;
    variances.hinge_axis_decorrelation_time = 1.0f;

    for (int i = 0; i < 1000; ++i) {
        // Create sensor map with some simple state state.
        const int numSensors = 2;
        OrientationFromMotion ofm(numSensors, jsm, variances);
        // Random orientation, approx -g*1s velocity, random bias.
        for (int i = 0; i < numSensors; ++i)
            ofm.sensors[i] += (Vector9f() << Vector3f::Random(), Vector3f::Random() * 10.0f, Vector3f::Random()).finished();
        // Create angular velocity measurements
        const Vector3f ws[numSensors] = { Vector3f::Random(), Vector3f(0.0f, 1.0f, 0.0f)};

        MatrixXf jacobianHxDummy(3 * (numSensors+jsm.numJoints+jsm.hinges.size()) + 1, SensorState::DOF * numSensors);
        MatrixXf jacobianHODummy(3 * (numSensors+jsm.numJoints+jsm.hinges.size()) + 1, 3 * numSensors); // Assuming every sensor is somehow involved with at least one joint, which is always the case in every sane configuration.
        const float epsilon = 1e-4f;
        const int stateDof = numSensors * SensorState::DOF;

        // Numerically compute jacobian of measurement model derived by state.
        MatrixXf jacobianHxNumeric(3 * (numSensors+jsm.numJoints+jsm.hinges.size()) + 1, SensorState::DOF * numSensors);
        SensorState splus[numSensors];
        SensorState sminus[numSensors];
        for (int d = 0; d < stateDof; ++d) {
            VectorXf delta = VectorXf::Zero(stateDof);
            delta(d) = epsilon;
            arraybplus(splus, ofm.sensors, delta, numSensors);
            arraybplus(sminus, ofm.sensors, -delta, numSensors);
            VectorXf expectedPlus(3 * (numSensors+jsm.numJoints+jsm.hinges.size()) + 1);
            VectorXf expectedMinus(3 * (numSensors+jsm.numJoints+jsm.hinges.size()) + 1);
            ofm.measurement_model(splus, expectedPlus, jacobianHxDummy, jacobianHODummy, ws);
            ofm.measurement_model(sminus, expectedMinus, jacobianHxDummy, jacobianHODummy, ws);
            jacobianHxNumeric.col(d) = (expectedPlus - expectedMinus) / (epsilon * 2.0f);
        }
        VectorXf expectedMeasDummy(3 * (numSensors + jsm.numJoints + jsm.hinges.size()) + 1);

        MatrixXf jacobianHoNumeric(3 * (numSensors+jsm.numJoints + jsm.hinges.size()) + 1, 3 * numSensors);
        const auto wbplus = [&numSensors](Vector3f * const w2, const Vector3f * const w1, const VectorXf& delta) {
            for (int s = 0; s < numSensors; ++s)
                w2[s] = w1[s] + delta.segment<3>(3*s);
        };
        const int measDof = 3 * numSensors;
        Vector3f wplus[numSensors];
        Vector3f wminus[numSensors];
        for (int d = 0; d < measDof; ++d) {
            VectorXf delta = VectorXf::Zero(measDof);
            delta(d) = epsilon;
            VectorXf expectedPlus(3 * (numSensors+jsm.numJoints+jsm.hinges.size()) + 1);
            VectorXf expectedMinus(3 * (numSensors+jsm.numJoints+jsm.hinges.size()) + 1);
            wbplus(wplus, ws, delta);
            wbplus(wminus, ws, -delta);
            ofm.measurement_model(ofm.sensors, expectedPlus, jacobianHxDummy, jacobianHODummy, wplus);
            ofm.measurement_model(ofm.sensors, expectedMinus, jacobianHxDummy, jacobianHODummy, wminus);
            jacobianHoNumeric.col(d) = (expectedPlus - expectedMinus) / (epsilon * 2.0f);
        }

        MatrixXf jacobianHx(3 * (numSensors+jsm.numJoints+jsm.hinges.size()) + 1, SensorState::DOF * numSensors);
        MatrixXf jacobianHo(3 * (numSensors+jsm.numJoints+jsm.hinges.size()) + 1, 3 * numSensors);
        ofm.measurement_model(expectedMeasDummy, jacobianHx, jacobianHo, ws);

        // The derivative of the constraint on the z-component of the orientation axis
        // of the first sensor (sensor 0) is approximated via the BCH formula and tested extra.
        const MatrixXf jacobianHx_diff_top = jacobianHx.topRows(3 * (numSensors+jsm.numJoints+jsm.hinges.size())) - jacobianHxNumeric.topRows(3 * (numSensors+jsm.numJoints+jsm.hinges.size()));
        const MatrixXf jacobianHx_diff_bottom = jacobianHx.bottomRows<1>() - jacobianHxNumeric.bottomRows<1>();

        if (jacobianHx_diff_top.norm() > 0.1f) {
            BOOST_ERROR("Jacobian by state is wrong.");
        }
        if (jacobianHx_diff_bottom.norm() > 0.01f) {
            BOOST_ERROR("First sensor orientation Jacobian by state is wrong.");
        }
        const MatrixXf jacobianHo_diff = jacobianHo - jacobianHoNumeric;
        if (jacobianHo_diff.norm() > 0.05f) {
            BOOST_ERROR("Jacobian by anuglar velocities inaccurate.");
        }
    }
}

BOOST_AUTO_TEST_CASE(dynamic_update_simple_test)
{
    std::stringstream ss;
    ss << "1 joints" << std::endl
       << "0 0 1.0 0.0 0.0 S 0.0 0.0 0.0" << std::endl
       << "0 1 -1.0 0.0 0.0 S 0.0 0.0 0.0" << std::endl;
    JointSensorMap jsm(ss);
    BOOST_CHECK(jsm.numJoints == 1);

    struct Variances variances;
    variances.acceleration = 1.0f;
    variances.acceleration_density = 1.0f;
    variances.angular_velocity = 1.0f;
    variances.angular_velocity_density = 1.0f;
    variances.joint_velocity_difference_variance = 1.0f;
    variances.joint_velocity_difference_decorrelation_time = 1.0f;
    variances.velocity_variance = 1.0f;
    variances.velocity_decorrelation_time = 1.0f;
    variances.z_variance = 1.0f;
    variances.z_decorrelation_time = 1.0f;
    variances.hinge_axis_variance = 0.125f*0.125f;
    variances.hinge_axis_decorrelation_time = 1.0f;

    constexpr int numSensors = 2;
    SensorState initial_states[numSensors];
    const MatrixXf initial_covariances[numSensors] = { Matrix9f::Identity(), Matrix9f::Identity() };
    OrientationFromMotion ofm(numSensors, jsm, variances);
    ofm.initialize(initial_states, initial_covariances);

    // Very simple test, which should lead to a proper covariance structure.
    const Vector3f gravity(0.0f, 0.0f, -9.81f);
    const Vector3f movement(1.0f, 0.0f, 0.0f);
    const float time = 0.5f;
    Accumulate as[2] = { Accumulate(Eigen::Quaternionf::Identity(), (movement - gravity) * time, static_cast<unsigned int>(time * 1000000)),
                         Accumulate(Eigen::Quaternionf::Identity(), (movement - gravity) * time, static_cast<unsigned int>(time * 1000000))};
    ofm.covariance.setIdentity();

    ofm.dynamic_update(as);

    // Check that all sensor state components have non-zero covariance.
    for (int i = 0; i < numSensors; ++i) {
        const Matrix9f sensorCov = ofm.covariance.block<SensorState::DOF, SensorState::DOF>(i * SensorState::DOF, i * SensorState::DOF);
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                const bool nonzeroblock = !sensorCov.block<3, 3>(j * 3, k * 3).isApprox(Matrix3f::Zero());
                BOOST_CHECK(nonzeroblock);
            }
        }
    }

    // Check that the dynamic model did not introduce any sensor-sensor correlation
    for (int i = 0; i < numSensors; ++i)
        for (int j = 0; j < numSensors; ++j)
            if (i != j) {
                const bool blockzero = ofm.covariance.block<SensorState::DOF, SensorState::DOF>(i * SensorState::DOF, j * SensorState::DOF).isApprox(Matrix9f::Zero());
                BOOST_CHECK(blockzero);
            }

    // Check the sensor components.
    const Vector9f expected = (Vector9f() << Vector3f::Zero(), movement * time, Vector3f::Zero()).finished();
    SensorState unit;
    for (int i = 0; i < numSensors; ++i) {
        const Vector9f d = ofm.sensors[i] - unit;
        BOOST_CHECK(d.isApprox(expected));
    }
}

static Accumulate create_accumulate(const Vector3f& w, const Vector3f& b, const Vector3f& a_world, const float deltaT, const unsigned steps, bf::ofstream& log)
{
    Eigen::Quaternionf Q_devInWorld = Eigen::Quaternionf::Identity();
    Accumulate a(Q_devInWorld, Vector3f::Zero(), deltaT * steps);

    for (unsigned i = 0; i < steps; ++i) {
        Q_devInWorld = Q_devInWorld * Eigen::AngleAxisf(w.norm() * deltaT, w.normalized());
        const Vector3f a_dev = Q_devInWorld.inverse() * a_world;
        const Vector3f wbiased = w + b;
        Eigen::Quaternionf deltaQ_biased; deltaQ_biased = Eigen::AngleAxisf(wbiased.norm() * deltaT, wbiased.normalized());
        a.Q *= deltaQ_biased;
        a.v += a.Q * a_dev * deltaT;
        if (log.is_open())
            log << a.v(0) << ' ' << a.v(1) << ' ' << a.v(2) << std::endl;
    }

    return a;
}

BOOST_AUTO_TEST_CASE(dynamic_update_bias_removal_test)
{
    std::stringstream ss;
    ss << "1 joints" << std::endl
       << "0 0 1.0 0.0 0.0 S 0.0 0.0 0.0" << std::endl
       << "0 1 -1.0 0.0 0.0 S 0.0 0.0 0.0" << std::endl;
    JointSensorMap jsm(ss);
    BOOST_CHECK(jsm.numJoints == 1);

    struct Variances variances;
    variances.acceleration = 1.0f;
    variances.acceleration_density = 1.0f;
    variances.angular_velocity = 1.0f;
    variances.angular_velocity_density = 1.0f;
    variances.joint_velocity_difference_variance = 1.0f;
    variances.joint_velocity_difference_decorrelation_time = 1.0f;
    variances.velocity_variance = 1.0f;
    variances.velocity_decorrelation_time = 1.0f;
    variances.z_variance = 1.0f;
    variances.z_decorrelation_time = 1.0f;
    variances.hinge_axis_variance = 0.125f*0.125f;
    variances.hinge_axis_decorrelation_time = 1.0f;
    const Vector3f gravity(0.0f, 0.0f, -9.81f);
    const int numSensors = 2;

    const auto thetest = [&](const Vector3f& omega, const Vector3f& bias, const int logid) -> void {
        const std::string logidstr = std::to_string(logid);
        const bf::path velocity_biased_path = bf::current_path()/("dynamic_update_bias_removal_test_velocity_biased_"+logidstr+".dat");
        const bf::path velocity_unbiased_path = bf::current_path()/("dynamic_update_bias_removal_test_velocity_unbiased_"+logidstr+".dat");
        const bf::path velocity_dynamic_update_path = bf::current_path()/("dynamic_update_velocity_results_"+logidstr+".dat");
        bf::ofstream velocity_biased(velocity_biased_path);
        bf::ofstream velocity_unbiased(velocity_unbiased_path);
        bf::ofstream velocity_results(velocity_dynamic_update_path);

        OrientationFromMotion ofm_knownbias(numSensors, jsm, variances);
        OrientationFromMotion ofm_zerobias(numSensors, jsm, variances);

        const Accumulate unbiased = create_accumulate(omega, Vector3f::Zero(), -gravity, 1.0f/100.0f, 50, velocity_unbiased);
        const Accumulate biased = create_accumulate(omega, bias, -gravity, 1.0f/100.0f, 50, velocity_biased);
        for (int i = 0; i < numSensors; ++i)
            ofm_knownbias.sensors[i].b = bias;

        const Accumulate as[2] = { unbiased, biased };
        ofm_zerobias.dynamic_update(as);
        ofm_knownbias.dynamic_update(as);

        const Vector3f& zb_unbiased = ofm_zerobias.sensors[0].v;
        const Vector3f& zb_biased = ofm_zerobias.sensors[1].v;
        const Vector3f& b_biased = ofm_knownbias.sensors[1].v;
        velocity_results << "# Zero-Bias, unbiased" << std::endl
        << zb_unbiased(0) << ' ' << zb_unbiased(1) << ' ' << zb_unbiased(2) << ' ' << "unbiased" << std::endl;
        velocity_results << "# Zero-Bias, biased" << std::endl
        << zb_unbiased(0) << ' ' << zb_biased(1) << ' ' << zb_biased(2) << ' ' << "biased" << std::endl;
        velocity_results << "# Known Bias, biased" << std::endl
        << b_biased(0) << ' ' << b_biased(1) << ' ' << b_biased(2) << ' ' << "biased+corrected" << std::endl;

        const Vector3f err_uncorrected = zb_biased - zb_unbiased;
        const Vector3f err_corrected = b_biased - zb_unbiased;
        if (!(err_corrected.norm() <= err_uncorrected.norm()))
            BOOST_CHECK(err_corrected.norm() < 1e-3f);
    };

    thetest(Vector3f(10.0f/180.0f*M_PI, 30.0f/180.0f*M_PI, 0.0f), Vector3f::Random().normalized() * M_PI/180.0f, 0);
    for (int i = 1; i < 100; ++i)
        thetest(Vector3f::Random().normalized() * 30.0f/180.0f*M_PI, Vector3f::Random().normalized() * M_PI/2.0f/180.0f, i);
}

BOOST_AUTO_TEST_CASE(measurement_update_test)
{
    std::stringstream ss;
    ss << "1 joints" << std::endl
       << "0 0 1.0 0.0 0.0 S 0.0 0.0 0.0" << std::endl
       << "0 1 -1.0 0.0 0.0 S 0.0 0.0 0.0" << std::endl;
    JointSensorMap jsm(ss);
    BOOST_CHECK(jsm.numJoints == 1);

    const Variances variances = {
        .acceleration = (300e-6f * 9.81f * 300e-6f * 9.81f) * 200.0f /**< Bosch documentation */,
        .acceleration_density = (300e-6f * 9.81f * 300e-6f * 9.81f) * 100.0f,
        .angular_velocity = (0.007f * M_PI/180.0f)*(0.007f * M_PI/180.0f) * 200.0f,
        .angular_velocity_density = (0.007f * M_PI/180.0f)*(0.007f * M_PI/180.0f),
        .gyro_bias = (1.0f/180.0f*M_PI)*(1.0f/180.0f*M_PI) / 3600.0f, /**< 1 deg per hour gyro bias change. */
        .joint_velocity_difference_variance = 0.01f,
        .joint_velocity_difference_decorrelation_time = 0.1f,
        .velocity_variance = 0.1f,
        .velocity_decorrelation_time = 5.0f,
        .z_variance = 1.0f,
        .z_decorrelation_time = 300.0f,
        .hinge_axis_variance = 0.125f*0.125f,
        .hinge_axis_decorrelation_time = 0.1f
    };
    const Vector3f gravity(0.0f, 0.0f, -9.81f);
    const int numSensors = 2;
    const float deltaT = 0.005f;
    const Vector3f omega(0.0f, 0.0f, 20.0f/180.0f * M_PI);

    const float joint_variance = variances.joint_velocity_difference_variance * (1.0f + 2.0f * variances.joint_velocity_difference_decorrelation_time / deltaT);
    const float velocity_variance = variances.velocity_variance * (1.0f + 2.0f * variances.velocity_decorrelation_time / deltaT);
    const float z_variance = variances.z_variance * (1.0f + 2.0f * variances.z_decorrelation_time / deltaT);
    const VectorXf measurement_variance_diag = (VectorXf(3*(numSensors+jsm.numJoints)+1)
                                                << VectorXf(3*numSensors).setConstant(velocity_variance),
                                                VectorXf(3*jsm.numJoints).setConstant(joint_variance), z_variance).finished();
    const MatrixXf measurement_variance = measurement_variance_diag.asDiagonal();
    const Eigen::LLT<MatrixXf> measurement_variance_solver = measurement_variance.llt();

    OrientationFromMotion ofm(numSensors, jsm, variances);
    const SensorLocation& sl1 = ofm.jsm.sensors[0].front();
    const SensorLocation& sl2 = ofm.jsm.sensors[0].back();
    // Initialize orientation from motion to a circular motion
    SensorState initial_states[numSensors];
    initial_states[sl1.sensorId].v = omega.cross(-sl1.jointInSensor); // '-' because of sensorInJoint
    initial_states[sl2.sensorId].v = omega.cross(-sl2.jointInSensor); // '-' because of sensorInJoint
    const MatrixXf initial_covariances[numSensors] = { Matrix9f::Identity(), Matrix9f::Identity() };
    ofm.initialize(initial_states, initial_covariances);

    const Vector3f motionaccel1 = omega.cross(ofm.sensors[sl1.sensorId].v);
    const Vector3f motionaccel2 = omega.cross(ofm.sensors[sl2.sensorId].v);

    bf::ofstream noOutput;

    Vector3f ws[2] = { omega, omega };
    Accumulate as[2] = { create_accumulate(ws[0], Vector3f::Zero(), motionaccel1-gravity, deltaT, 20, noOutput),
                         create_accumulate(ws[1], Vector3f::Zero(), motionaccel2-gravity, deltaT, 20, noOutput) };

    ofm.dynamic_update(as);
    const Eigen::LLT<MatrixXf> ofm_covariance_solver = ofm.covariance.llt();

    MatrixXf dummyJacobianHx, dummyJacobianHo;
    VectorXf expectedMeas(3 * (numSensors+jsm.numJoints) + 1);

    // Grid sampling
    const int numSamplesPerDim = 50;
    const int numSamples = 2 * numSamplesPerDim + 1;
    const int dimension = numSensors * SensorState::DOF;
    float parameter_x[dimension][numSamples];
    float sample_px[dimension][numSamples];
    float sample_pxz[dimension][numSamples];
    float weights[dimension][numSamples];
    SensorState sampleStates[dimension][numSamples][numSensors];
    for (int d = 0; d < dimension; ++d) {
        VectorXf delta(dimension);
        delta.setZero();
        delta(d) = 1.0f;
        MatrixXf samples(dimension, numSamples);
        samples.setZero();
        const float variance_d = ofm.covariance(d, d);
        const float stddev_d = sqrt(variance_d);
        for (int k = 0; k < numSamplesPerDim; ++k) {
            parameter_x[d][k] = static_cast<float>(k+1)/static_cast<float>(numSamplesPerDim) * 2.0f * stddev_d;
            samples.col(k) = delta * parameter_x[d][k];
            parameter_x[d][numSamplesPerDim + k] = -parameter_x[d][k];
            samples.col(numSamplesPerDim + k) = delta * parameter_x[d][numSamplesPerDim + k];
        }
        parameter_x[d][numSamples-1] = 0.0f;

        for (int i = 0; i < numSamples; ++i) {
            sample_px[d][i] = expf(-0.5f * samples.col(i).dot(ofm_covariance_solver.solve(samples.col(i))));
            arraybplus(sampleStates[d][i], ofm.sensors, samples.col(i), numSensors);
            ofm.measurement_model(sampleStates[d][i], expectedMeas, dummyJacobianHx, dummyJacobianHo, ws);
            weights[d][i] = expf(-0.5f * (expectedMeas.dot(measurement_variance_solver.solve(expectedMeas))));
            sample_pxz[d][i] = weights[d][i] * sample_px[d][i];
        }
    }
    // Do the update
    ofm.measurement_update(ws, deltaT);
    // Compute posterior probabilities from updated filter distribution
    const Eigen::LLT<MatrixXf> updated_ofm_covariance_solver = ofm.covariance.llt();
    float updated_sample_px[dimension][numSamples]; // Should be sample_p(x|z) = sample_pxz
    VectorXf diff(numSensors * SensorState::DOF);
    float sampledInUpdated[dimension]; // Normalization conversion factor.
    for (int d = 0; d < dimension; ++d) {
        for (int i = 0; i < numSamples; ++i) {
            arraybminus(diff, sampleStates[d][i], ofm.sensors, numSensors);
            updated_sample_px[d][i] = expf(-0.5f * diff.dot(updated_ofm_covariance_solver.solve(diff)));
        }
        sampledInUpdated[d] = updated_sample_px[d][numSamples-1] / sample_pxz[d][numSamples-1];
    }

    // Check and write results.
    for (int d = 0; d < dimension; ++d) {
        const bf::path distribution_dimension_path = bf::current_path()/("measurement_update_test_dimension_"+std::to_string(d)+".dat");
        bf::ofstream distribution_dimension_log(distribution_dimension_path);
        for (int i = 0; i < numSamples; ++i) {
            if (updated_sample_px[d][i] > 1e-2f && sample_pxz[d][i] > 1e-2f)
                BOOST_CHECK_CLOSE(sample_pxz[d][i] * sampledInUpdated[d], updated_sample_px[d][i], 20.0f);
            distribution_dimension_log << parameter_x[d][i] << ' ' << sample_px[d][i] << ' ' << sample_pxz[d][i] * sampledInUpdated[d] << ' ' << updated_sample_px[d][i] << std::endl;
        }
    }
}

struct AccumulateBoxOps
{
    enum { DOF = 6 };
    static Vector6f minus(const Accumulate& lhs, const Accumulate& rhs)
    {
        Vector3f q;
        const Matrix3f deltaQ = (rhs.Q.inverse() * lhs.Q).toRotationMatrix();
        Rot::logm(q, deltaQ);
        return (Vector6f() << q, (lhs.v - rhs.v)).finished();
    }
    static Accumulate plus(const Accumulate& lhs, const Vector6f& rhs)
    {
        Eigen::Quaternionf Q;
        Q = Eigen::AngleAxisf(rhs.segment<3>(0).norm(), rhs.segment<3>(0).normalized());
        return Accumulate(lhs.Q * Q, lhs.v + rhs.segment<3>(3), lhs.duration);
    }
};

struct ParameterBoxOps
{
    enum { DOF = 6 };
    static Vector6f minus(const Vector6f& lhs, const Vector6f& rhs) { return lhs-rhs; };
    static Vector6f plus(const Vector6f& lhs, const Vector6f& rhs) { return lhs+rhs; };
};

Accumulate update_accumulate(const Accumulate& a, const Vector6f& inertialmeas,
                             const float deltaT)
{
    Eigen::Quaternionf dQ;
    dQ = Eigen::AngleAxisf(inertialmeas.segment<3>(0).norm() * deltaT, inertialmeas.segment<3>(0).normalized());
    Eigen::Vector3f dv = dQ * inertialmeas.segment<3>(3) * deltaT;
    return a * Accumulate(dQ, dv, deltaT);
}

BOOST_AUTO_TEST_CASE(accumulate_covariance_approximation)
{
    /* Define the accumulate and a covariance matrix to integrate. Later on,
       we will compute the covariance approximation for the complete accumulate
       and see how well it fits to the propagated covariance matrix. */
    const float deltaT = 0.01f;
    Matrix6f cov;
    cov.setZero();
    Accumulate a;

    /* Both integrating and approximating the covariance need the sensor variances,
       which we define here. Since the approximation is a method of the OFM, we
       instantiate the OFM with the variances we also use to integrate the
       accumulate covariance matrix. */
    struct Variances variances;
    variances.acceleration = 1.0f;
    variances.acceleration_density = 1.0f * deltaT;
    variances.angular_velocity = 1.0f;
    variances.angular_velocity_density = 1.0f * deltaT;
    variances.joint_velocity_difference_variance = 1.0f;
    variances.joint_velocity_difference_decorrelation_time = 1.0f;
    variances.velocity_variance = 1.0f;
    variances.velocity_decorrelation_time = 1.0f;
    variances.z_variance = 1.0f;
    variances.z_decorrelation_time = 1.0f;
    variances.hinge_axis_variance = 0.125f*0.125f;
    variances.hinge_axis_decorrelation_time = 1.0f;
    OrientationFromMotion ofm(1, JointSensorMap(0), variances);

    /* For 10 steps, which corresponds to the 10Hz output of our estimator,
       we now integrate the accumulate. */
    for (unsigned k = 0; k < 10; ++k) {
        /* The measurement vector. u=[w' a']'. */
        const Vector6f u = (Vector6f() << 100.0f/180.0f*M_PI, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f).finished();
        /* The measurement covariance. */
        Matrix6f Sigma_u(Matrix6f::Zero());
        Sigma_u.topLeftCorner<3,3>() = Matrix3f::Identity() * (variances.angular_velocity);
        Sigma_u.bottomRightCorner<3,3>() = Matrix3f::Identity() * (variances.acceleration);
        /* Jacobians of the Accumulate update. */
        Matrix6f A, B;
        compute_jacobian<AccumulateBoxOps, ParameterBoxOps>(boost::bind(update_accumulate, a, _1, deltaT), u, B);
        compute_jacobian<AccumulateBoxOps, AccumulateBoxOps>(boost::bind(update_accumulate, _1, u, deltaT), a, A);
        /* Use the Jacobians to propagate the covariance through the accumulate update. */
        cov = A * cov * A.transpose() + B * Sigma_u * B.transpose();
        /* Compute the new accumulate. */
        a = update_accumulate(a, u, deltaT);
    }

    /* Compute the covariance approximation. */
    Matrix6f cov_approx;
    ofm.covariance_for_accumulate(cov_approx, a);

    /* Check the difference. The the biggest variance in the difference of the
       propagated and approximated covariance should not exceed 10% of the
       corresponding variance in the propagated covariance matrix:
       |u' (cov_approx-cov) u| < (u' cov u) / 10, for any u with |u|=1. */
    const Matrix6f deltacov = cov_approx - cov;
    Eigen::SelfAdjointEigenSolver<Matrix6f> es(deltacov);
    const Vector6f evals = es.eigenvalues();
    const Matrix6f evecs = es.eigenvectors();
    int maxidx, minidx;
    const float maxev = evals.maxCoeff(&maxidx);
    const float minev = evals.minCoeff(&minidx);
    float maxdeltavar; int maxdeltavar_idx;
    if (std::abs(maxev) > std::abs(minev)) {
        maxdeltavar = std::abs(maxev);
        maxdeltavar_idx = maxidx;
    } else {
        maxdeltavar = std::abs(minev);
        maxdeltavar_idx = minidx;
    }
    const float maxvar = evecs.col(maxdeltavar_idx).transpose() * cov * evecs.col(maxdeltavar_idx);
    BOOST_CHECK_LE(maxdeltavar, maxvar / 10.0f);
}
