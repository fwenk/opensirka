/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __ORIENTATION_FROM_MOTION_H_
#define __ORIENTATION_FROM_MOTION_H_

#include <JointSensorMap.h>
#include "single_sensor_state.h"
#include "ofm_log.h"

typedef LIR::JointSensorMap<float> JointSensorMap;

class OrientationFromMotion {
    const int numSensors;
    const int maxIterations;

    /** Distribution parameters. */
    SensorState *sensors;
    MatrixXf covariance;
    bool initialized;

    /** Jacobians of the dynamic and measurement model. */
    MatrixXf jacobianA; /**< Dynamic model derived by the state. */
    MatrixXf jacobianB; /**< Dynamic model derived by the accumulates. */
    MatrixXf jacobianHx; /**< Measurement model derived by the state. */
    MatrixXf jacobianHo; /**< Measurement model derived by the angular velocities. */

    VectorXf expectedMeasurement; /**< Attribute to hold the result of the measurement model. */

    /** Sensor configuration */
    const JointSensorMap jsm;
    const Variances variances;
    /** The table for the covariances if the hinge axis in the 3d-unit-vector representations.
     *  hingeAxisCovariances[k] stores the covariance of hinge k both in successor-sensor-
     *  as well as in predecessor-sensor-coordinates. */
    HingeAxixCovariances *hingeAxisCovariances;

    /** Logging */
    std::unique_ptr<LogStreams> logStreams;

    void dynamic_model(SensorState *sensors, MatrixXf& jacobianA, MatrixXf& jacobianB, const Accumulate * const as);
    void measurement_model(const SensorState * const sensors, VectorXf& expectedMeas, MatrixXf& jacobianHx, MatrixXf& jacobianHo, const Vector3f * const ws) const;
    void compute_hinge_constraint_measurement_variance(Eigen::Ref<MatrixXf> cov, const float timescale);
public:
    OrientationFromMotion(int numSensors, const JointSensorMap& jsm, const Variances& variances, int maxIterations = 1);
    ~OrientationFromMotion();
    bool vectorization() const;

    static void single_dynamic_model(SensorState& s, Matrix9f& jacobianA, Matrix9f6& jacobianB, const Accumulate& a);
    void dynamic_model(MatrixXf& jacobianA, MatrixXf& jacobianB, const Accumulate * const as);
    void measurement_model(VectorXf& expectedMeas, MatrixXf& jacobianHx, MatrixXf& jacobianHo, const Vector3f * const ws) const;

    void covariance_for_accumulate(Matrix6f& covariance, const Accumulate& a);

    void initialize(const SensorState initialStates[], const MatrixXf initialSensorCovariances[]);
    void initialize_from_first_accumulates(const Accumulate * const as);
    void dynamic_update(const Accumulate * const as);
    void measurement_update(const Vector3f * const ws, const float deltaT);

    void set_log_path(const std::string& log_path);
    void log(const float time);

    Matrix3f get_orientation(unsigned sid) const;

    // Test cases are friends.
    friend struct dynamicmodel_jacobians_trivial;
    friend struct dynamicmodel_jacobians;
    friend struct measurementmodel_jacobians;
    friend struct dynamic_update_simple_test;
    friend struct dynamic_update_bias_removal_test;
    friend struct measurement_update_test;
};

#endif // __ORIENTATION_FROM_MOTION_H_
