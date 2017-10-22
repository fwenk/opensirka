#ifndef __SENSOR_LOCATION_H_
#define __SENSOR_LOCATION_H_

/*
 * Copyright (c) 2017 Felix Wenk <felixwenk@googlemail.com>
 */

#include <Eigen/Dense>
#include <istream>
#include <algorithm>

namespace LIR
{
enum JointType
{
    hinge, /**< Joint with one rotary degrees of freedom. */
    spherical /**< Joint with three independent rotary degrees of freedom. */
};

/**
 * The location of a joint relative to a sensor.
 */
template<typename real>
class SensorLocation
{
public:
    typedef Eigen::Matrix<real, 3, 1> Vector3;

    Vector3 jointInSensor; /**< The position of the joint in sensor coordinates (the r's in the paper) */
    int sensorId; /**< Number of the sensor. */
    JointType type;
    /**
     * If this object refers to a hinge, hingeAxisInSensor contains the axis
     * of rotation in sensor coordinates. If this object does not refer to a hinge,
     * the axis is zero.
     */
    Vector3 hingeAxisInSensor;

    /** Default constructor */
    SensorLocation() : jointInSensor(Vector3::Zero()), sensorId(-1), type(spherical) {}
    /** Constructor for spherical joints. */
    SensorLocation(const Vector3& jointInSensor, const int sensorId)
    : jointInSensor(jointInSensor), sensorId(sensorId), type(spherical)
    {}
    /** Constructor for hinges. */
    SensorLocation(const Vector3& jointInSensor, const int sensorId,
                   const Vector3& hingeAxisInSensor)
    : jointInSensor(jointInSensor), sensorId(sensorId), type(hinge), hingeAxisInSensor(hingeAxisInSensor)
    {}

    void setFromStream(std::istream& is)
    {
        char type;
        is >> sensorId >> jointInSensor.x() >> jointInSensor.y() >> jointInSensor.z() >> type
           >> hingeAxisInSensor.x() >> hingeAxisInSensor.y() >> hingeAxisInSensor.z();
        switch (type) {
            case 'h':
            case 'H':
                this->type = hinge;
                break;
            case 's':
            case 'S':
                this->type = spherical;
                break;
            default:
                throw "Encountered impossible joint type while reading SensorLocation.";
        }
    }
};

/**
 * Representation of a joint.
 */
template<typename real>
class Joint
{
private:
    int jointId;
    SensorLocation<real> predecessorLocation;
    SensorLocation<real> successorLocation;
public:
    Joint() {}
    Joint(int jointId, const SensorLocation<real>& predecessor, const SensorLocation<real>& successor)
    : jointId(jointId), predecessorLocation(predecessor), successorLocation(successor)
    {}
    const SensorLocation<real>& predecessor() const { return predecessorLocation; }
    SensorLocation<real>& predecessor() { return predecessorLocation; }
    const SensorLocation<real>& front() const { return predecessor(); }
    const SensorLocation<real>& successor() const { return successorLocation; }
    SensorLocation<real>& successor() { return successorLocation; }
    const SensorLocation<real>& back() const { return successor(); }
    int getJointId() const { return jointId; }
};
}

#endif // __SENSOR_LOCATION_H_
