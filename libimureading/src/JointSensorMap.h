/*
 * Copyright (c) 2017 Felix Wenk <felixwenk@googlemail.com>
 */
#ifndef __JOINT_SENSOR_MAP_H_
#define __JOINT_SENSOR_MAP_H_

#include <vector>
#include <sstream>

#include "SensorLocation.h"

namespace LIR
{
    /**
     * Representation of structure, i.e. the kinematic tree, of a skeleton.
     * A skeleton is a collection of joints connecting adjacent bodies.
     */
    template<typename real>
    class JointSensorMap
    {
    public:
        typedef SensorLocation<real> SensorLocation;
        typedef Joint<real> Joint;

        int numJoints;
        std::vector<Joint> sensors; /**< sensors[j] has the sensors connected to joint j. */
        /** Index of hinges.
          * hinges[k] is the joint index of hinge k.
          * I.e. sensors[hinges[k]] is the list of SensorLocations of bodies connected
          * to hinge k. */
        std::vector<int> hinges;

        JointSensorMap(const JointSensorMap& other)
        : numJoints(other.numJoints), sensors(other.sensors)
        {}

        JointSensorMap(std::istream& is)
        {
            std::string line;
            std::getline(is, line);
            std::stringstream ss(line);
            ss >> numJoints;
            sensors.reserve(numJoints);
            while (!is.eof()) {
                std::getline(is, line);
                if (line.empty())
                    break;
                SensorLocation predecessor;
                int jointid;
                {
                    std::stringstream ss(line);
                    ss >> jointid;
                    predecessor.setFromStream(ss);
                }
                std::getline(is, line);
                SensorLocation successor;
                {
                    std::stringstream ss(line);
                    int testid;
                    ss >> testid;
                    if (testid != jointid)
                        throw "The joint sensor map is ill structured.";
                    successor.setFromStream(ss);
                }
                sensors.push_back(Joint(jointid, predecessor, successor));
            }

            if (numJoints != sensors.size())
                throw "Did not read the number of joints specified at the beginning of the file.";

            for (unsigned j = 0; j < numJoints; ++j)
                if (sensors[j].predecessor().type == JointType::hinge)
                    hinges.push_back(j);
        }
    };
};

#endif // __JOINT_SENSOR_MAP_H_
