/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#define BOOST_TEST_MODULE "test_loadresolutions"

#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/log/trivial.hpp>

#include "../src/IMUReading.h"

namespace bf = boost::filesystem;

BOOST_AUTO_TEST_CASE(load_resolutions)
{
    BOOST_LOG_TRIVIAL(info) << "Loading some resolutions.";

    const bf::path tmpdir = bf::temp_directory_path();
    const bf::path tmpfile = bf::unique_path(tmpdir/"resolutions.txt-%%%%%%");
    BOOST_LOG_TRIVIAL(debug) << "Using temporary file: " << tmpfile;

    const struct LIR::Resolutions original = {
        .accelerometer = 13.37,
        .gyroscope = 73.31
    };
    LIR::save_resolutions(tmpfile.string(), original);
    const struct LIR::Resolutions loaded = LIR::load_resolutions(tmpfile.string());
    bf::remove(tmpfile);

    BOOST_CHECK_EQUAL(original.accelerometer, loaded.accelerometer);
    BOOST_CHECK_EQUAL(original.gyroscope, loaded.gyroscope);
}
