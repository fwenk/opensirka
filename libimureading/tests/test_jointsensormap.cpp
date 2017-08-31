/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2017 Felix Wenk <felixwenk@googlemail.com>
 */
#define BOOST_TEST_MODULE "JointSensorMap"

#include "../src/SensorLocation.h"
#include "../src/JointSensorMap.h"

#include <fstream>

#include <boost/test/unit_test.hpp>
#include <boost/log/trivial.hpp>

using namespace Eigen;

BOOST_AUTO_TEST_CASE(load_jsm)
{
    /* Get the path of the test data file. */
    if (boost::unit_test::framework::master_test_suite().argc < 2)
        BOOST_FAIL("Path of test data file not given.");
    const char *filename = boost::unit_test::framework::master_test_suite().argv[1];
    BOOST_LOG_TRIVIAL(info) << "Loading test data file: " << filename;

    std::ifstream istream(filename);
    typedef LIR::JointSensorMap<float> JointSensorMap;
    JointSensorMap jsm(istream);

    /* Assert some properties known from the test file. */
    BOOST_ASSERT(jsm.sensors.size() == 14);
    BOOST_ASSERT(jsm.sensors[12].predecessor().sensorId == 9);
    BOOST_ASSERT(jsm.sensors[12].successor().sensorId == 14);
    BOOST_ASSERT(jsm.hinges.size() == 4);
}
