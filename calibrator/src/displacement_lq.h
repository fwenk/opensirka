/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __DISPLACEMENT_LQ_H_
#define __DISPLACEMENT_LQ_H_

#include "TimedSensorState.h"
#include "joint_sensor_map.h"
#include <IMUAccumulate.h>

struct StandardDeviations {
    double acceleration;
    double acceleration_density;
    double angular_velocity;
    double angular_velocity_density;
    double joint_velocity_difference;
    double joint_velocity_difference_decorrelation_time;
    double joint_axis_difference;
    double joint_axis_difference_decorrelation_time;
    double velocity;
    double velocity_decorrelation_time;
    double symmetric_body_length_difference;
};

void compute_initial_hinge_axes_guess(
        std::vector<std::shared_ptr<LIR::IMUAccumulates>> readings,
        JointSensorMap& jsm, const StandardDeviations& stddevs);

void calibrate_displacements(
        std::vector<std::shared_ptr<LIR::IMUAccumulates>> readings,
        std::vector<TimedSensorStateRun>& imuTrajectories,
        JointSensorMap& jsm,
        const struct StandardDeviations& stddevs,
        const std::list<Symmetry>& symmetries);

#endif // __DISPLACEMENT_LQ_H_
