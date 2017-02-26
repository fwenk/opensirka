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
#include <vector>
#include <unordered_map>
#include <iostream>
#include <memory>

using namespace Eigen;

SensorLocation::SensorLocation(const Vector3d& jointInSensor, const int sensorId)
: jointInSensor(jointInSensor), sensorId(sensorId),
  type(JointType::spherical), hingeAxisInSensor(Vector3d::Zero())
{}

SensorLocation::SensorLocation(const Vector3d& jointInSensor, const int sensorId,
                               const Vector3d& hingeAxisInSensor)
: jointInSensor(jointInSensor), sensorId(sensorId),
  type(JointType::hinge), hingeAxisInSensor(hingeAxisInSensor)
{}

JointSensorMap::JointSensorMap(const int numJoints)
: numJoints(numJoints), sensors(new JointSensors[numJoints])
{}

JointSensorMap::JointSensorMap(const JointSensorMap& other)
: numJoints(other.numJoints), sensors(new JointSensors[numJoints])
{
    for (int j = 0; j < numJoints; ++j)
        sensors[j] = JointSensors(other.sensors[j]);
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
: numJoints(numJointsFromStream(is)), sensors(new JointSensors[numJoints])
{
    while (!is.eof()) {
        std::string line;
        std::getline(is, line);
        if (line.empty())
            continue;
        std::stringstream ss(line);
        Vector3d jointInSensor, hingeAxisInSensor;
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
}

void JointSensorMap::save(std::ostream &os) const
{
    os << numJoints << " joints" << std::endl;
    for (int j = 0; j < numJoints; ++j) {
        for (const SensorLocation& sl : sensors[j]) {
            const char type = sl.type == JointType::hinge ? 'H' : 'S';
            const Eigen::Vector3d& r = sl.jointInSensor;
            const Eigen::Vector3d& a = sl.hingeAxisInSensor;
            os << j << ' ' << sl.sensorId << ' ' << r.x() << ' ' << r.y() << ' ' << r.z()
               << ' ' << type << ' ' << a.x() << ' ' << a.y() << ' ' << a.z() << std::endl;
        }
    }
}

void JointSensorMap::rotateToWorldCoordinates(const std::vector<Eigen::Quaterniond>& sensorInWorld)
{
    for (int j = 0; j < numJoints; ++j) {
        for (SensorLocation& sl : sensors[j]) {
            const Eigen::Quaterniond& Q = sensorInWorld[sl.sensorId];
            sl.jointInSensor = Q * sl.jointInSensor;
            if (sl.type == JointType::hinge)
                sl.hingeAxisInSensor = Q * sl.hingeAxisInSensor;
        }
    }
}

static void drawArrow(Vector3d& from, Vector3d& to, std::ostream& stream)
{
    stream << "draw((" << from.x() << ", " << from.y() << ", " << from.z()
           << ") -- ("
           << to.x() << ", " << to.y() << ", " << to.z()
           << "), arrow = Arrow3());" << std::endl;
}

static void drawSphere(Vector3d& center, double size, const std::string& color, std::ostream& stream)
{
    stream << "draw(shift((" << center.x() << ", " << center.y() << ", " << center.z()
           << ")) * scale3(" << size << ") * unitsphere, surfacepen=" << color << ");" << std::endl;
}

static void drawAxis(const Vector3d& from, const Vector3d& to, const std::string& color, std::ostream& stream)
{
    stream << "draw((" << from.x() << ", " << from.y() << ", " << from.z()
           << ") -- ("
           << to.x() << ", " << to.y() << ", " << to.z()
           << "), " << color << "+linewidth(1));" << std::endl;
}

void JointSensorMap::exportToAsymptote(const std::vector<Eigen::Quaterniond>& sensorInWorld,
                                       std::ostream& asyfile)
{
    asyfile.precision(5);
    asyfile << std::fixed;
    /* Write an asymptote program to plot this jsm. The asymptote program will
       draw arrows from each sensor s to each joint adjacent to s. */
    using namespace std;
    JointSensorMap jsm_inWorld(*this);
    jsm_inWorld.rotateToWorldCoordinates(sensorInWorld);

    /* Since the joint locations are stored only relative to sensors in the JSM,
       we keep track of where we've drawn each sensor and joint. We then draw the
       elements not yet drawn relative to those saved positions. */
    const int numSensors = numJoints + 1;
    vector<shared_ptr<Vector3d>> joint_pos_world(numJoints);
    vector<shared_ptr<Vector3d>> sensor_pos_world(numSensors);
    sensor_pos_world[0].reset(new Vector3d(0.0, 0.0, 0.0));
    list<int> undrawn_joints(numJoints);
    {
        int j = 0;
        generate(undrawn_joints.begin(), undrawn_joints.end(), [&j]{ return j++; });
    }

    /* Write beginning of the program. This is mostly defaults for configuration
       options which control how the JSM is plotted. */
    asyfile << "settings.outformat = \"pdf\";" << endl
            << "settings.render = 0;" << endl
            << "settings.prc = true;" << endl
            << "import three;" << endl
            << "size(10cm, 10cm);" << endl << endl;

    while (!undrawn_joints.empty()) {
        /* Find drawable joint. */
        list<int>::iterator j_it = find_if(undrawn_joints.begin(), undrawn_joints.end(),
                [&sensor_pos_world, &jsm_inWorld](const int j) -> bool {
                    const JointSensorMap::JointSensors& js = jsm_inWorld.sensors[j];
                    const int predidx = js.predecessor().sensorId;
                    const int succidx = js.successor().sensorId;
                    return sensor_pos_world[predidx] || sensor_pos_world[succidx];
                });
        assert (j_it != undrawn_joints.end());
        cout << "Plotting joint " << *j_it << endl;

        /* Compute potentially missing positions in world. */
        assert(!(joint_pos_world[*j_it]));
        const SensorLocation& ploc = jsm_inWorld.sensors[*j_it].predecessor();
        const SensorLocation& sloc = jsm_inWorld.sensors[*j_it].successor();
        if (sensor_pos_world[sloc.sensorId]) {
            joint_pos_world[*j_it].reset(
                new Vector3d(*sensor_pos_world[sloc.sensorId] + sloc.jointInSensor));
            sensor_pos_world[ploc.sensorId].reset(
                new Vector3d(*joint_pos_world[*j_it] - ploc.jointInSensor));
        } else if (sensor_pos_world[ploc.sensorId]) {
            joint_pos_world[*j_it].reset(
                new Vector3d(*sensor_pos_world[ploc.sensorId] + ploc.jointInSensor));
            sensor_pos_world[sloc.sensorId].reset(
                new Vector3d(*joint_pos_world[*j_it] - sloc.jointInSensor));
        } else {
            cerr << "Pah. Found a drawable sensor location, which turned out not to be drawable. There's a bug somewhere." << endl;
        }

        /* Draw arrow from successor to joint. */
        drawArrow(*sensor_pos_world[sloc.sensorId], *joint_pos_world[*j_it], asyfile);
        /* Draw arrow from predecessor to joint. */
        drawArrow(*sensor_pos_world[ploc.sensorId], *joint_pos_world[*j_it], asyfile);

        /* Remove joint from undrawn joints. */
        undrawn_joints.erase(j_it);
    }

    /* Draw hinge axes and spheres for joints and sensors. */
    for (int j = 0; j < numJoints; ++j) {
        const SensorLocation& ploc = jsm_inWorld.sensors[j].predecessor();
        const SensorLocation& sloc = jsm_inWorld.sensors[j].successor();
        if (ploc.type == JointType::hinge) {
            const double length = 0.06;
            const Vector3d& c = *joint_pos_world[j];
            drawAxis(c - ploc.hingeAxisInSensor * length, c, "orange", asyfile);
            assert (sloc.type == JointType::hinge);
            drawAxis(c, c + sloc.hingeAxisInSensor * length, "blue", asyfile);
        }
        drawSphere(*joint_pos_world[j], 0.02, "red", asyfile);
    }
    for (int s = 0; s < numSensors; ++s)
        drawSphere(*sensor_pos_world[s], 0.015, "green", asyfile);
}

JointSensorMap::~JointSensorMap()
{
    delete[] sensors;
}
