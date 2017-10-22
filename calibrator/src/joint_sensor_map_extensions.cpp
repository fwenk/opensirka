/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH Author: Felix Wenk
 * Copyright (c) 2017 Felix Wenk <felixwenk@googlemail.com>
 */
#include "joint_sensor_map_extensions.h"
#include <vector>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <iterator>

using namespace Eigen;

// MARK: Joint Sensor Map implementation

static void save_sensor_location(const int joint_id, const JointSensorMap::SensorLocation& sl, std::ostream& os)
{
    const char type = sl.type == LIR::JointType::hinge ? 'H' : 'S';
    const Eigen::Vector3d& r = sl.jointInSensor;
    const Eigen::Vector3d& a = sl.hingeAxisInSensor;
    os << joint_id << ' ' << sl.sensorId << ' ' << r.x() << ' ' << r.y() << ' ' << r.z()
                   << ' ' << type        << ' ' << a.x() << ' ' << a.y() << ' ' << a.z() << std::endl;
}

void JointSensorMap::save(std::ostream &os) const
{
    os << numJoints << " joints" << std::endl;
    for (const Joint& j : sensors) {
        save_sensor_location(j.getJointId(), j.predecessor(), os);
        save_sensor_location(j.getJointId(), j.successor(), os);
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

    const vector<JointSensorMap::Joint>& jsmsensors = this->sensors;
    while (!undrawn_joints.empty()) {
        /* Find drawable joint. */
        list<int>::iterator j_it = find_if(undrawn_joints.begin(), undrawn_joints.end(),
                [&sensor_pos_world, &jsmsensors](const int j) -> bool {
                    const JointSensorMap::Joint& sj = jsmsensors[j];
                    const int predidx = sj.predecessor().sensorId;
                    const int succidx = sj.successor().sensorId;
                    return sensor_pos_world[predidx] || sensor_pos_world[succidx];
                });
        assert (j_it != undrawn_joints.end());
        cout << "Plotting joint " << *j_it << endl;

        const SensorLocation& ploc = sensors[*j_it].predecessor();
        const SensorLocation& sloc = sensors[*j_it].successor();
        const SensorLocation& known = sensor_pos_world[ploc.sensorId] ? ploc : sloc;
        const SensorLocation& unknown = sensor_pos_world[sloc.sensorId] ? ploc : sloc;
        joint_pos_world[*j_it].reset(
            new Vector3d(*sensor_pos_world[known.sensorId] + sensorInWorld[known.sensorId] * known.jointInSensor)
        );
        sensor_pos_world[unknown.sensorId].reset(
            new Vector3d(*joint_pos_world[*j_it] - sensorInWorld[unknown.sensorId] * unknown.jointInSensor)
        );

        /* Draw arrow from successor to joint. */
        drawArrow(*sensor_pos_world[sloc.sensorId], *joint_pos_world[*j_it], asyfile);
        /* Draw arrow from predecessor to joint. */
        drawArrow(*sensor_pos_world[ploc.sensorId], *joint_pos_world[*j_it], asyfile);

        /* Remove joint from undrawn joints. */
        undrawn_joints.erase(j_it);
    }

    /* Draw hinge axes and spheres for joints and sensors. */
    for (const Joint& j : sensors) {
        const int jointId = j.getJointId();
        const SensorLocation& ploc = j.predecessor();
        const SensorLocation& sloc = j.successor();
        if (ploc.type == LIR::JointType::hinge) {
            const double length = 0.06;
            const Vector3d& c = *joint_pos_world[jointId];
            const Vector3d hpredInWorld = sensorInWorld[ploc.sensorId] * ploc.hingeAxisInSensor;
            drawAxis(c - hpredInWorld * length, c, "orange", asyfile);
            assert (sloc.type == JointType::hinge);
            const Vector3d hsuccInWorld = sensorInWorld[sloc.sensorId] * sloc.hingeAxisInSensor;
            drawAxis(c, c + hsuccInWorld * length, "blue", asyfile);
        }
        drawSphere(*joint_pos_world[jointId], 0.02, "red", asyfile);
    }
    for (int s = 0; s < numSensors; ++s)
        drawSphere(*sensor_pos_world[s], 0.015, "green", asyfile);
}

// MARK: JointSymmetries implementation


Symmetry::Symmetry(const std::string& string)
{
    std::stringstream ss(string);
    ss >> a.body_id >> a.preceeding_joint_id >> a.succeeding_joint_id
       >> b.body_id >> b.preceeding_joint_id >> b.succeeding_joint_id;
}

class Line : public std::string
{
public:
    typedef std::istream_iterator<Line> Iterator;
private:
    friend std::istream& operator>>(std::istream& is, Line& line)
    {
        return std::getline(is, line);
    }
};

JointSymmetries::JointSymmetries()
{}

JointSymmetries::JointSymmetries(std::istream& is)
{
    Line::Iterator it(is), end;
    for (; it != end; ++it)
        if ((*it)[0] != '#' && !std::isspace((*it)[0]))
            symmetries.push_back(Symmetry(*it));
}
