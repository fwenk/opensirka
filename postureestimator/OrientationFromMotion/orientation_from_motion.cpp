/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include "orientation_from_motion.h"

#include <Eigen/Dense>
#include <boost/log/trivial.hpp>
#include <MathHelper/expm.h>
#include <MathHelper/crossx.h>
#include <MathHelper/sphere.h>

OrientationFromMotion::OrientationFromMotion(int numSensors, const JointSensorMap& jsm, const Variances& variances, int maxIterations)
: numSensors(numSensors), maxIterations(maxIterations), initialized(false), sensors(new SensorState[numSensors]),
  covariance(MatrixXf::Identity(numSensors * SensorState::DOF, numSensors * SensorState::DOF)),
  jacobianA(numSensors * SensorState::DOF, numSensors * SensorState::DOF),
  jacobianB(numSensors * SensorState::DOF, numSensors * Accumulate::DOF),
  jacobianHx(3 * (numSensors + jsm.numJoints + jsm.hinges.size()) + 1, numSensors * SensorState::DOF),
  jacobianHo(3 * (numSensors + jsm.numJoints + jsm.hinges.size()) + 1, numSensors * 3),
  expectedMeasurement(3 * (numSensors + jsm.numJoints + jsm.hinges.size()) + 1),
  jsm(jsm), variances(variances), hingeAxisCovariances(new HingeAxixCovariances[jsm.hinges.size()]),
  logStreams(nullptr)
{
    /** Compute the unit-duration covariance of the 3d-representations of the hinge
        axes in predecessor-sensor- and in successor-sensor-coordinates. */

    /** Variance of the hinge axis in boxplus-parameters. */
    const Eigen::Matrix2f axisSphereCov = Eigen::Matrix2f::Identity() * variances.hinge_axis_variance;

    for (int k = 0; k < jsm.hinges.size(); ++k) {
        const SensorLocation& ploc = jsm.sensors[jsm.hinges[k]].front();
        const SensorLocation& sloc = jsm.sensors[jsm.hinges[k]].back();
        assert (ploc.type == JointType::hinge);
        assert (sloc.type == JointType::hinge);
        /* Compute the Jacobians of the boxplus operation. The boxplus operation
           is used to map small changes of the point on the sphere to the 3d-overparameterization.
           The Jacobians are used to propagate the covariance of the 2d-uncertainty
           through the boxplus operation, to get the approximate covariance in 3d in
           sensor coordinates. */
        const Eigen::Matrix<float, 3, 2> bplusJacobianPredecessor = Sphere::dotbplus(ploc.hingeAxisInSensor);
        const Eigen::Matrix<float, 3, 2> bplusJacobianSuccessor   = Sphere::dotbplus(sloc.hingeAxisInSensor);
        /* Do the covariance propagation, both for the representation in predecessor-sensor-
           and successor-sensor-coordinates. */
        hingeAxisCovariances[k].covInPredecessor = bplusJacobianPredecessor * axisSphereCov * bplusJacobianPredecessor.transpose();
        hingeAxisCovariances[k].covInSuccessor   = bplusJacobianSuccessor * axisSphereCov * bplusJacobianSuccessor.transpose();
    }
}

OrientationFromMotion::~OrientationFromMotion()
{
    delete[] sensors;
    delete[] hingeAxisCovariances;
}

bool OrientationFromMotion::vectorization() const
{
#ifdef EIGEN_VECTORIZE
    return true;
#else
    return false;
#endif
}

void OrientationFromMotion::single_dynamic_model(SensorState &s, Matrix9f& jacobianA, Matrix9f6& jacobianB, const Accumulate &a)
{
    s.single_dynamic_model(a, jacobianA, jacobianB);
}

void OrientationFromMotion::dynamic_model(SensorState *sensors, MatrixXf &jacobianA, MatrixXf &jacobianB, const Accumulate *const as)
{
    jacobianA = MatrixXf::Zero(numSensors * SensorState::DOF, numSensors * SensorState::DOF);
    jacobianB = MatrixXf::Zero(numSensors * SensorState::DOF, numSensors * Accumulate::DOF);
    Matrix9f blockA;
    Matrix9f6 blockB;
    for (int i = 0; i < numSensors; ++i) {
        sensors[i].single_dynamic_model(as[i], blockA, blockB);
        jacobianA.block<SensorState::DOF, SensorState::DOF>(i * SensorState::DOF, i * SensorState::DOF) = blockA;
        jacobianB.block<SensorState::DOF, Accumulate::DOF>(i * SensorState::DOF, i * Accumulate::DOF) = blockB;
    }
}

void OrientationFromMotion::dynamic_model(MatrixXf& jacobianA, MatrixXf& jacobianB, const Accumulate * const as)
{
    dynamic_model(sensors, jacobianA, jacobianB, as);
}

void OrientationFromMotion::measurement_model(const SensorState * const sensors, VectorXf& expectedMeas, MatrixXf &jacobianHx, MatrixXf &jacobianHo, const Vector3f *const ws) const
{
    jacobianHo = MatrixXf::Zero(3*(numSensors+jsm.numJoints+jsm.hinges.size())+1, numSensors*3);
    jacobianHx = MatrixXf::Zero(3*(numSensors+jsm.numJoints+jsm.hinges.size())+1, SensorState::DOF*numSensors); // TODO: Remove.
    /* Compute expected 'measurements' and Jacobian of velocity prior. */
    for (int i = 0; i < numSensors; ++i) {
        expectedMeas.segment<3>(i * 3) = sensors[i].v;

        jacobianHx.block<3,3>(i * 3, i * SensorState::DOF + 3) = Matrix3f::Identity();
    }
    /* Compute expected 'measurement' and Jacobian entries of joint-velocity-prior. */
    for (int j = 0; j < jsm.numJoints; ++j) {
        assert(jsm.sensors[j].size() == 2); // TODO: Implement support for multiple sensors per body later.
        const SensorLocation& ploc = jsm.sensors[j].front();
        const SensorLocation& sloc = jsm.sensors[j].back();
        const Vector3f& r_p = ploc.jointInSensor;
        const Vector3f& w_p = ws[ploc.sensorId] - sensors[ploc.sensorId].b;
        const Vector3f& r_s = sloc.jointInSensor;
        const Vector3f& w_s = ws[sloc.sensorId] - sensors[sloc.sensorId].b;

        const Vector3f psi_p = w_p.cross(r_p);
        const Vector3f psi_s = w_s.cross(r_s);
        const Vector3f J_p = sensors[ploc.sensorId].v + sensors[ploc.sensorId].Q * psi_p;
        const Vector3f J_s = sensors[sloc.sensorId].v + sensors[sloc.sensorId].Q * psi_s;
        expectedMeas.segment<3>(3 * (numSensors + j)) = J_s - J_p;

        // Derivative by orientations
        jacobianHx.block<3,3>(3*(numSensors+j), ploc.sensorId * SensorState::DOF) = sensors[ploc.sensorId].Q * crossx(psi_p);
        jacobianHx.block<3,3>(3*(numSensors+j), sloc.sensorId * SensorState::DOF) = -sensors[sloc.sensorId].Q * crossx(psi_s);
        // Derivative by velocities
        jacobianHx.block<3,3>(3*(numSensors+j), ploc.sensorId * SensorState::DOF + 3) = -Matrix3f::Identity();
        jacobianHx.block<3,3>(3*(numSensors+j), sloc.sensorId * SensorState::DOF + 3) = Matrix3f::Identity();
        // Derivatives by the bias (same as the negative of the derivatives by the angular velocities)
        jacobianHx.block<3,3>(3*(numSensors+j), ploc.sensorId * SensorState::DOF + 6) = -sensors[ploc.sensorId].Q * crossx(ploc.jointInSensor);
        jacobianHx.block<3,3>(3*(numSensors+j), sloc.sensorId * SensorState::DOF + 6) = sensors[sloc.sensorId].Q * crossx(sloc.jointInSensor);

        // Derivative of measurement model by the angular velocities.
        // Velocities by angular velocities: Always 0.
        // The last orientation component by angular velocities, also always 0.
        // Jj by angular velocities:
        jacobianHo.block<3,3>(3*(numSensors+j), sloc.sensorId * 3) = -sensors[sloc.sensorId].Q * crossx(sloc.jointInSensor);
        jacobianHo.block<3,3>(3*(numSensors+j), ploc.sensorId * 3) = sensors[ploc.sensorId].Q * crossx(ploc.jointInSensor);
    }
    /* Compute expected 'measurement' and Jacobian entries of hinge constraints. */
    for (int k = 0; k < jsm.hinges.size(); ++k) {
        const SensorLocation& ploc = jsm.sensors[jsm.hinges[k]].front();
        const SensorLocation& sloc = jsm.sensors[jsm.hinges[k]].back();
        assert (ploc.type == JointType::hinge);
        assert (sloc.type == JointType::hinge);
        const Vector3f h_s = sensors[sloc.sensorId].Q * sloc.hingeAxisInSensor;
        const Vector3f h_p = sensors[ploc.sensorId].Q * ploc.hingeAxisInSensor;
        expectedMeas.segment<3>(3 * (numSensors+jsm.numJoints+k)) = h_s - h_p;
        /* Compute the derivative by the orientations. All other derivatives are zero. */
        jacobianHx.block<3,3>(3*(numSensors+jsm.numJoints+k), ploc.sensorId * SensorState::DOF) = sensors[ploc.sensorId].Q * crossx(ploc.hingeAxisInSensor);
        jacobianHx.block<3,3>(3*(numSensors+jsm.numJoints+k), sloc.sensorId * SensorState::DOF) = -sensors[sloc.sensorId].Q * crossx(sloc.hingeAxisInSensor);
    }
    // y-component of the sensors x-vector in world coordinates should be 0: y'*Q*x = Q(1,0)
    const unsigned headingPriorIdx = numSensors > 2 ? 2 : numSensors/2;
    const Matrix3f& headingPriorQ = sensors[headingPriorIdx].Q;
    expectedMeas(3 * (numSensors + jsm.numJoints + jsm.hinges.size())) = headingPriorQ(1,0);
    // Derivates of the last component are all zero except for the headingPriorIdx's sensor orientation.
    jacobianHx.block<1,3>(3*(numSensors+jsm.numJoints+jsm.hinges.size()), SensorState::DOF * headingPriorIdx)
        = (Vector3f() << 0.0f, -headingPriorQ(1,2), headingPriorQ(1,1)).finished();
}

void OrientationFromMotion::measurement_model(VectorXf& expectedMeas, MatrixXf &jacobianHx, MatrixXf &jacobianHo, const Vector3f *const ws) const
{
    measurement_model(sensors, expectedMeas, jacobianHx, jacobianHo, ws);
}

void OrientationFromMotion::covariance_for_accumulate(Matrix6f& covariance, const Accumulate& a)
{
    const float a_duration_sec = a.durationSeconds();
    const Matrix3f vx = crossx(a.v);
    covariance << Matrix3f::Identity() * a_duration_sec, vx * a_duration_sec / 2.0f,
        vx.transpose() * a_duration_sec / 2.0f, vx.transpose() * vx * a_duration_sec / 3.0f;
    covariance *= variances.angular_velocity_density;
    covariance.bottomRightCorner<3, 3>() +=
        Matrix3f::Identity() * variances.acceleration_density * a_duration_sec;
}

void OrientationFromMotion::initialize(const SensorState initialStates[], const MatrixXf initialSensorCovariances[])
{
    for (int i = 0; i < numSensors; ++i) {
        this->sensors[i] = initialStates[i];
        this->covariance.block<SensorState::DOF,SensorState::DOF>(i * SensorState::DOF, i * SensorState::DOF) = initialSensorCovariances[i];
    }
    initialized = true;
}

void OrientationFromMotion::initialize_from_first_accumulates(const Accumulate * const as)
{
    Matrix3f proto_Q_covar = Matrix3f::Zero();
    proto_Q_covar(0,0) = 5.0f/180.0f * M_PI * 5.0f/180.0f * M_PI;
    proto_Q_covar(1,1) = 5.0f/180.0f * M_PI * 5.0f/180.0f * M_PI;
    proto_Q_covar(2,2) = 25.0f/180.0f * M_PI * 25.0f/180.0f * M_PI;
    const Matrix3f proto_v_covar = Matrix3f::Identity();
    const Matrix3f proto_b_covar = Matrix3f::Identity() * (0.014f * M_PI / 180.0f)*(0.014f * M_PI / 180.0f);

    for (int i = 0; i < numSensors; ++i) {
        const Vector3f& v_inImu = as[i].v;
        Vector3f v_inWorld;
        v_inWorld << 0.0f, 0.0f, v_inImu.norm();
        Eigen::Quaternionf q_imuInWorld;
        q_imuInWorld.setFromTwoVectors(v_inImu, v_inWorld);
        sensors[i].Q = q_imuInWorld.toRotationMatrix();
        sensors[i].v = Vector3f::Zero();
        sensors[i].b = Vector3f::Zero();
        covariance.block<3, 3>(i * SensorState::DOF, i * SensorState::DOF) = sensors[i].Q * proto_Q_covar * sensors[i].Q.transpose();
        covariance.block<3, 3>(i * SensorState::DOF + 3, i * SensorState::DOF + 3) = proto_v_covar;
        covariance.block<3, 3>(i * SensorState::DOF + 6, i * SensorState::DOF + 6) = proto_b_covar;
    }
    initialized = true;
}

void OrientationFromMotion::dynamic_update(const Accumulate *const as)
{
    if (!initialized) {
        initialize_from_first_accumulates(as);
        return;
    }

    // TODO: Optimize by expanding more by hand, avoiding numerous multiplications by 0.

    // Dynamic model which updates the state.
    dynamic_model(jacobianA, jacobianB, as);
    // TODO: There's probably a better way for block-diagonal matrices in eigen.
    MatrixXf sigma_u(numSensors * Accumulate::DOF, numSensors * Accumulate::DOF);
    sigma_u.setZero();
    Matrix6f accCov;
    for (int i = 0; i < numSensors; ++i) {
        covariance_for_accumulate(accCov, as[i]);
        sigma_u.block<Accumulate::DOF, Accumulate::DOF>(i * Accumulate::DOF, i * Accumulate::DOF) = accCov;
    }
    covariance = jacobianA * covariance * jacobianA.transpose() + jacobianB * sigma_u * jacobianB.transpose();
    // Add process noise for the bias
    for (int i = 0; i < numSensors; ++i) {
        const float deltaT = as[i].durationSeconds();
        covariance.block<3,3>(SensorState::DOF * i + 6, SensorState::DOF * i + 6)
                += Matrix3f::Identity() * variances.gyro_bias * deltaT;
    }
}

void OrientationFromMotion::measurement_update(const Vector3f *const ws, const float deltaT)
{
    /* Compute the variances of the measurements, a.k.a. priors spread over time deltaT */
    const float joint_variance = variances.joint_velocity_difference_variance * (1.0f + 2.0f * variances.joint_velocity_difference_decorrelation_time / deltaT);
    const float velocity_variance = variances.velocity_variance * (1.0f + 2.0f * variances.velocity_decorrelation_time / deltaT);
    const float z_variance = variances.z_variance * (1.0f + 2.0f * variances.z_decorrelation_time / deltaT);
    const VectorXf measurement_variance_diag = (VectorXf(3*(numSensors+jsm.numJoints+jsm.hinges.size())+1)
        << VectorXf(3*numSensors).setConstant(velocity_variance),
           VectorXf(3*jsm.numJoints).setConstant(joint_variance),
           VectorXf::Zero(3*jsm.hinges.size()), z_variance).finished();
    MatrixXf measurement_variance = measurement_variance_diag.asDiagonal();
    /* Compute covariance of hinge constraints. */
    compute_hinge_constraint_measurement_variance(measurement_variance.block(3*(numSensors+jsm.numJoints), 3*(numSensors+jsm.numJoints), 3*jsm.hinges.size(), 3*jsm.hinges.size()),
                                                  1.0f + 2.0f * variances.hinge_axis_decorrelation_time / deltaT);
    /* Compute the measurement model and its Jacobians. */
    measurement_model(expectedMeasurement, jacobianHx, jacobianHo, ws);
    /* Make the pseudo-measurement of the global heading equal to the current global heading. */
    expectedMeasurement.tail<1>()[0] = 0.0f;
    SensorState startstates[numSensors];
    for (unsigned k = 0; k < numSensors; ++k)
        startstates[k] = sensors[k];

    const MatrixXf identity = MatrixXf::Identity(SensorState::DOF * numSensors, SensorState::DOF * numSensors);
    VectorXf deltaState(SensorState::DOF * numSensors);
    MatrixXf K, covZX;
    const MatrixXf covYY = jacobianHo * variances.angular_velocity * jacobianHo.transpose() + measurement_variance;
    for (unsigned k = 0;; ++k) {
        covZX = jacobianHx * covariance;
        const MatrixXf covZZ = covZX * jacobianHx.transpose() + covYY;

        const Eigen::LDLT<MatrixXf> cholCovZZ = covZZ.ldlt();
        K = cholCovZZ.solve(covZX).transpose();
        VectorXf increment = - K * expectedMeasurement;
        arraybminus(deltaState, startstates, sensors, numSensors);
        increment += (identity - K * jacobianHx) * deltaState;
        arraybplus(sensors, increment, numSensors);
        float squaredIncrementNorm = increment.squaredNorm();
        if (squaredIncrementNorm < 1e-4f || k == maxIterations-1)
            break;
        measurement_model(expectedMeasurement, jacobianHx, jacobianHo, ws);
        expectedMeasurement.tail<1>()[0] = 0.0f;
    }
    covariance -= K * covZX;
    const MatrixXf covT = covariance.transpose();
    covariance += covT;
    covariance /= 2.0f;
}

void OrientationFromMotion::compute_hinge_constraint_measurement_variance(Eigen::Ref<MatrixXf> cov, const float timescale)
{
    /* Compute the block-diagonal covariance matrix of the hinge constraint.
       The precomputed covariances of the hinge axes in sensor coordinates
       are scaled with time and rotated into the world frame, according to the
       constraint definition in the measurement model.
       This is done per hinge, and per-hinge-constraint-covariance is then stamped
       into the diagonal of the measurement covariance matrix of the all hinge
       conrstraints combined. */
    for (int k = 0; k < jsm.hinges.size(); ++k) {
        /* Get the data for the current constraint. */
        const std::list<SensorLocation>&  hingeSensors   = jsm.sensors[jsm.hinges[k]];
        const SensorLocation&             predSensor     = hingeSensors.front();
        const SensorLocation&             succSensor     = hingeSensors.back();
        const Matrix3f&                   Q_predInWorld  = sensors[predSensor.sensorId].Q;
        const Matrix3f&                   Q_succInWorld  = sensors[succSensor.sensorId].Q;
        const Matrix3f&                   hingeCovInPred = hingeAxisCovariances[k].covInPredecessor;
        const Matrix3f&                   hingeCovInSucc = hingeAxisCovariances[k].covInSuccessor;
        /* Compute the covariance. */
        cov.block<3,3>(3*k, 3*k) = (Q_predInWorld * hingeCovInPred * Q_predInWorld.transpose()
            + Q_succInWorld * hingeCovInSucc * Q_succInWorld.transpose()) * timescale;
    }
}

void OrientationFromMotion::set_log_path(const std::string& log_path)
{
    /* Check if the passed log path is sensible. If it is empty, do not try
       write logs. Otherwise logs are intended to be written, so we try to
       set up the logging objects. */
    if (log_path.empty()) {
        BOOST_LOG_TRIVIAL(warning) << "Log path set to the empty string. Not logging.";
        return;
    }

    logStreams.reset(new LogStreams(log_path, numSensors));
}

void OrientationFromMotion::log(const float time)
{
    if (logStreams) {
        logStreams->logState(sensors, time);
        logStreams->logCovariance(covariance, time);
    }
}

Matrix3f OrientationFromMotion::get_orientation(unsigned int sid) const
{
    return sensors[sid].Q;
}
