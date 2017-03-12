/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __OFM_LOG_H_
#define __OFM_LOG_H_

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <vector>
#include <memory>

#include "single_sensor_state.h"

namespace bf = boost::filesystem;

class SensorLogStream {
    const bf::path logDirectory;
    // The streams.
    bf::ofstream orientation;
    bf::ofstream velocity;
    bf::ofstream bias;
    bf::ofstream orientationStddev;
    bf::ofstream velocityStddev;
    bf::ofstream biasStddev;

public:
    SensorLogStream(const bf::path& logDirectory, const int sensorId);
    void logState(const SensorState& state, const float time);
    void logCovariance(const Eigen::Matrix<float, SensorState::DOF, SensorState::DOF>& covariance, const float time);
};

class LogStreams {
    const int numSensors;
    std::vector<std::unique_ptr<SensorLogStream>> logs;

public:
    LogStreams(const bf::path& logDirectory, const int numSensors);
    void logState(const SensorState * const state, const float time);
    void logCovariance(const Eigen::MatrixXf& covariance, const float time);
};


#endif // __OFM_LOG_H_
