/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include "joint_sensor_map.h"

SensorLocation::SensorLocation(const Vector3f& jointInSensor, const int sensorId)
: jointInSensor(jointInSensor), sensorId(sensorId),
  type(JointType::spherical), hingeAxisInSensor(Vector3f::Zero())
{}

SensorLocation::SensorLocation(const Vector3f& jointInSensor, const int sensorId,
                               const Vector3f& hingeAxisInSensor)
: jointInSensor(jointInSensor), sensorId(sensorId),
  type(JointType::hinge), hingeAxisInSensor(hingeAxisInSensor)
{}

JointSensorMap::JointSensorMap(const int numJoints)
: numJoints(numJoints), sensors(new std::list<SensorLocation>[numJoints])
{}

JointSensorMap::JointSensorMap(const JointSensorMap& other)
: numJoints(other.numJoints), sensors(new std::list<SensorLocation>[numJoints]),
  hinges(other.hinges)
{
    for (int j = 0; j < numJoints; ++j)
        sensors[j] = std::list<SensorLocation>(other.sensors[j]);
}

static int numJointsFromStream(std::istream& is)
{
    std::string line;
    std::getline(is, line);
    std::stringstream ss(line);
    int joints;
    std::string jointsstr;
    ss >> joints >> jointsstr;
    if (jointsstr != "joints")
        throw "Wrongly formatted joint sensor map.";
    if (joints < 0)
        throw "Number of joints can't be negative, but it is in the given map.";

    return joints;
}

JointSensorMap::JointSensorMap(std::istream& is)
: numJoints(numJointsFromStream(is)), sensors(new std::list<SensorLocation>[numJoints])
{
    while (!is.eof()) {
        std::string line;
        std::getline(is, line);
        if (line.empty())
            continue;
        std::stringstream ss(line);
        Vector3f jointInSensor, hingeAxisInSensor;
        int joint, sensor;
        char type;
        ss >> joint >> sensor >> jointInSensor.x() >> jointInSensor.y() >> jointInSensor.z() >> type
           >> hingeAxisInSensor.x() >> hingeAxisInSensor.y() >> hingeAxisInSensor.z();
        switch (type) {
            case 'H':
            case 'h':
                sensors[joint].push_back(SensorLocation(jointInSensor, sensor, hingeAxisInSensor));
                break;
            case 'S':
            case 's':
                sensors[joint].push_back(SensorLocation(jointInSensor, sensor));
                break;
            default:
                throw "Read unknown joint type!";
                break;
        }
    }

    hinges.clear();
    for (int j = 0; j < numJoints; ++j)
        if (sensors[j].front().type == JointType::hinge)
            hinges.push_back(j);
}

JointSensorMap::~JointSensorMap()
{
    delete[] sensors;
}
