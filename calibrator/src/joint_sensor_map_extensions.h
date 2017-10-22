/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __JOINT_SENSOR_MAP_EXTENSIONS_H_
#define __JOINT_SENSOR_MAP_EXTENSIONS_H_

#include <istream>
#include <list>
#include <vector>
#include <Eigen/Dense>

#include <JointSensorMap.h>

class JointSensorMap : public LIR::JointSensorMap<double>
{
public:
    /* Inherit base class constructors. */
    using LIR::JointSensorMap<double>::JointSensorMap;

    void save(std::ostream& os) const;

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

#endif // __JOINT_SENSOR_MAP_EXTENSIONS_H_
