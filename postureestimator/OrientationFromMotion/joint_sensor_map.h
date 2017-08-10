/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __JOINT_SENSOR_MAP_H_
#define __JOINT_SENSOR_MAP_H_

#include <istream>
#include <list>
#include <vector>
#include "ofm_types.h"

enum JointType { hinge, spherical };

class SensorLocation {
public:
    Vector3f jointInSensor; /**< The position of the joint in sensor coordinates (the r's in the paper) */
    int sensorId; /**< Number of the sensor. */
    JointType type; /**< Type of the joint whose position is stored in object. */
    /** If this object refers to a hinge, hingeAxisInSensor contains the axis
     * of rotation in sensor coordinates. If this object does not refere to a hinge,
     * the axis is zero. */
    Eigen::Vector3f hingeAxisInSensor;

    /** Constructor for spherical joints. */
    SensorLocation(const Vector3f& jointInSensor, const int sensorId);
    /** Constructor for hinges. */
    SensorLocation(const Eigen::Vector3f& jointInSensor, const int sensorId,
                   const Eigen::Vector3f& hingeAxisInSensor);
};

class JointSensorMap {
public:
    const int numJoints;
    std::list<SensorLocation> * const sensors; /**< sensors[j] is the list of SensorLocations on bodies connected to joint j. */
    /** Index of hinges.
     * hinges[k] is the joint index of hinge k.
     * I.e. sensors[hinges[k]] is the list of SensorLocations of bodies connected
     * to hinge k. */
    std::vector<int> hinges;

    JointSensorMap(const int numJoints);
    JointSensorMap(std::istream& is);
    JointSensorMap(const JointSensorMap& other);
    ~JointSensorMap();
};

#endif // __JOINT_SENSOR_MAP_H_
