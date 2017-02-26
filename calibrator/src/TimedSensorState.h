/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */

#ifndef __TIMED_SENSOR_STATE_H_
#define __TIMED_SENSOR_STATE_H_

#include <memory>
#include <list>
#include <ostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct TimedSensorState {
    const Eigen::Quaterniond q_imuInWorld;
    const Eigen::Vector3d v_inWorld;
    const double time;

    TimedSensorState(Eigen::Quaterniond& q_imuInWorld, Eigen::Vector3d& v_inWorld, double time)
    : q_imuInWorld(q_imuInWorld), v_inWorld(v_inWorld), time(time) {}

    void save(std::ostream& ostream) const;
};

typedef std::list<std::shared_ptr<TimedSensorState>> TimedSensorStateRun;
void saveTimedSensorStateRun(const TimedSensorStateRun& run, const std::string& filename);
std::shared_ptr<TimedSensorState> getStateAfterTime(const TimedSensorStateRun& run, const double seconds);
void saveOrientationAfterTime(const TimedSensorStateRun& run, const double seconds,
                              const std::string& filename);

#endif // __TIMED_SENSOR_STATE_H_
