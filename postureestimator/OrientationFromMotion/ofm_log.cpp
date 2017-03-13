/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include "ofm_log.h"

#include <boost/log/trivial.hpp>

// MARK: SensorLogStream implementation

static bf::path pathForSensorIdAndSuffix(const bf::path& logDirectory, const int sensorId, const std::string& suffix)
{
    return logDirectory/("sensor-"+std::to_string(sensorId)+suffix);
}

static void logVectorToStream(const float time, const Eigen::Vector3f& v, bf::ofstream& stream)
{
    stream << time << ' ' << v(0) << ' ' << v(1) << ' ' << v(2) << std::endl;
}

static void logMatrixToStream(const float time, const Eigen::Matrix3f& Q, bf::ofstream& stream)
{
    stream << time;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            stream << ' ' << Q(r, c);
    stream << std::endl;
}

SensorLogStream::SensorLogStream(const bf::path& logDirectory, const int sensorId)
: logDirectory(logDirectory),
  orientation(pathForSensorIdAndSuffix(logDirectory, sensorId, "-orientation.log")),
  velocity(pathForSensorIdAndSuffix(logDirectory, sensorId, "-velocity.log")),
  bias(pathForSensorIdAndSuffix(logDirectory, sensorId, "-bias.log")),
  orientationStddev(pathForSensorIdAndSuffix(logDirectory, sensorId, "-orientation_stddev.log")),
  velocityStddev(pathForSensorIdAndSuffix(logDirectory, sensorId, "-velocity_stddev.log")),
  biasStddev(pathForSensorIdAndSuffix(logDirectory, sensorId, "-bias_stddev.log"))
{}

void SensorLogStream::logState(const SensorState &state, const float time)
{
    logMatrixToStream(time, state.Q, orientation);
    logVectorToStream(time, state.v, velocity);
    logVectorToStream(time, state.b, bias);
}

void SensorLogStream::logCovariance(const Eigen::Matrix<float, SensorState::DOF, SensorState::DOF> &covariance, const float time)
{
    const Eigen::Matrix<float, SensorState::DOF, 1> variances = covariance.diagonal();
    const Eigen::Matrix<float, SensorState::DOF, 1> stddev = variances.cwiseSqrt();
    logVectorToStream(time, stddev.segment<3>(0), orientationStddev);
    logVectorToStream(time, stddev.segment<3>(3), velocityStddev);
    logVectorToStream(time, stddev.segment<3>(6), biasStddev);
}

// MARK: LogStreams implmentation

LogStreams::LogStreams(const bf::path& logDirectory, const int numSensors)
: numSensors(numSensors)
{
    logs.reserve(numSensors);
    for (int i = 0; i < numSensors; ++i)
        logs.push_back(std::unique_ptr<SensorLogStream>(new SensorLogStream(logDirectory, i)));
}

void LogStreams::logState(const SensorState *const state, const float time)
{
    for (int i = 0; i < numSensors; ++i)
        logs[i]->logState(state[i], time);
}

void LogStreams::logCovariance(const Eigen::MatrixXf &covariance, const float time)
{
    for (int i = 0; i < numSensors; ++i)
        logs[i]->logCovariance(covariance.block<SensorState::DOF, SensorState::DOF>(SensorState::DOF*i, SensorState::DOF*i), time);
}
