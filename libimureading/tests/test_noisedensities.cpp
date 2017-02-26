/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#define BOOST_TEST_MODULE "test_loadresolutions"

#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/log/trivial.hpp>
#include <fstream>

#include "../src/Variance.h"

namespace bf = boost::filesystem;

BOOST_AUTO_TEST_CASE(load_noisedensities)
{
    BOOST_LOG_TRIVIAL(info) << "I will make up noise densities, save them and try to load them back.";

    const bf::path tmpdir = bf::temp_directory_path();
    const bf::path tempfile = bf::unique_path(tmpdir/"noisedensities.txt-%%%%%%");
    BOOST_LOG_TRIVIAL(debug) << "Using temporary file: " << tempfile;

    {
        std::ofstream ofs(tempfile.string());
        ofs << "# IMU noise densities: 1: Accelerometer; 2: Gryoscope." << std::endl;
        ofs << "1 0.02" << std::endl << "2 0.01" << std::endl;
        ofs.close();
    }
    LIR::NoiseDensities nd(tempfile.string());
    BOOST_CHECK_EQUAL(0.02, nd.accelNoiseDensity);
    BOOST_CHECK_EQUAL(0.01, nd.gyroNoiseDensity);

    BOOST_CHECK((Eigen::Matrix3d::Identity() * 0.01 * 0.01 * 0.5).isApprox(nd.gyroCovariance(0.5)));
    BOOST_CHECK((Eigen::Matrix3d::Identity() * 0.02 * 0.02 * 0.5).isApprox(nd.accelCovariance(0.5)));

    bf::remove(tempfile);
}
