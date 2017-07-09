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
#include <Eigen/Dense>

enum JointType { hinge, spherical };

class SensorLocation {
public:
    Eigen::Vector3d jointInSensor; /**< The position of the joint in sensor coordinates (the r's in the paper) */
    int sensorId; /**< Number of the sensor. */
    JointType type; /**< Type of the joint whose position is stored in object. */
    /** If this object refers to a hinge, hingeAxisInSensor contains the axis
     * of rotation in sensor coordinates. If this object does not refere to a hinge,
     * the axis is zero. */
    Eigen::Vector3d hingeAxisInSensor;

    /** Constructor for spherical joints. */
    SensorLocation(const Eigen::Vector3d& jointInSensor, const int sensorId);
    /** Constructor for hinges. */
    SensorLocation(const Eigen::Vector3d& jointInSensor, const int sensorId,
                   const Eigen::Vector3d& hingeAxisInSensor);
};

class JointSensorMap {
public:
    class JointSensors : public std::list<SensorLocation>
    {
    public:
        const SensorLocation& predecessor() const { return front(); }
        const SensorLocation& successor() const   { return back(); }
    };
    const int numJoints;
    JointSensors * const sensors; /**< sensors[j] is the list of SensorLocations on bodies connected to joint j. */

    JointSensorMap(const int numJoints);
    JointSensorMap(std::istream& is);
    JointSensorMap(const JointSensorMap& other);
    ~JointSensorMap();

    void save(std::ostream& os) const;

    /**
     * Rotates every joint location and hinge axis into world coordinates.
     * To do so, the orientation of each sensor is passes in the parameter vector.
     * sensorInWorld[k] must be the orientation of sensor with id k in
     * in world coordinates.
     */
    void rotateToWorldCoordinates(const std::vector<Eigen::Quaterniond>& sensorInWorld);

    /**
     * Writes an asymptote program to plot this joint-sensor-map.
     */
    void exportToAsymptote(const std::vector<Eigen::Quaterniond>& sensorInWorld,
                           std::ostream& asyfile);
};

struct Symmetry {
    struct Body {
        /* Number of the body in the corresponding joint-sensor-map.
           This isn't strictly necessary for the computation itself,
           but provides for a useful consistency check; e.g. the
           body succeeding the joint preceeding this body should have
           the this body_id. */
        unsigned body_id;
        /* Number of the joint preceeding the body (or sensor, respectively). */
        unsigned preceeding_joint_id;
        /* Number of the joint succeeding the body (or sensor, respectively). */
        unsigned succeeding_joint_id;
    };

    /* Two bodies are symmetric if the distance between the preceeding
       joint and the succeeding joint of body a equals the distance
       of the corresponding joints of body b. */
    Symmetry(const std::string& string);

    Body a;
    Body b;
};

class JointSymmetries {
public:
    std::list<Symmetry> symmetries;
    JointSymmetries();
    JointSymmetries(std::istream& is);
    operator const std::list<Symmetry>&() const { return symmetries; }
};

#endif // __JOINT_SENSOR_MAP_H_
