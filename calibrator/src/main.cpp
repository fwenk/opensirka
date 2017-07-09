/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */

#include <iostream>
#include <string>
#include <fstream>
#include <boost/program_options.hpp>

#include <IMUAccumulate.h>
#include "joint_sensor_map.h"
#include "displacement_lq.h"
#include "TimedSensorState.h"

struct ProgramOptions {
    std::string jsm_file;
    std::string symmetry_file;
    std::string imu_data_dir;
    std::string calib_out;
    std::string prediction_out;
    int num_sensors;
} programOptions;

void parse_command_line_options(int argc, char **argv)
{
    namespace po = boost::program_options;

    po::options_description desc("Command Line Options");
    desc.add_options()
    ("help", "produce help message")
    ("imu_data", po::value<std::string>()->required(), "Directory to load the IMU accumulates from.")
    ("calib_out", po::value<std::string>()->required(), "Calibration output path.")
    ("prediction_out", po::value<std::string>(), "Orientation prediction output path.")
    ("num_sensors", po::value<int>()->required(), "Number of sensors to calibrate.")
    ("jsm_file", po::value<std::string>()->required(), "File with the joint-sensor-map.")
    ("symmetry_file", po::value<std::string>(), "File with the skeleton symmetry specification.");

    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).run();
    po::store(parsed, vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(0);
    }
    try {
        vm.notify();
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cout << desc << std::endl;
        exit(1);
    }

    programOptions.imu_data_dir = vm["imu_data"].as<std::string>();

    programOptions.calib_out = vm["calib_out"].as<std::string>();
    programOptions.prediction_out = "";
    if (vm.count("prediction_out"))
        programOptions.prediction_out = vm["prediction_out"].as<std::string>();
    programOptions.jsm_file = vm["jsm_file"].as<std::string>();
    programOptions.num_sensors = vm["num_sensors"].as<int>();
    if (vm.count("symmetry_file"))
        programOptions.symmetry_file = vm["symmetry_file"].as<std::string>();
}

static JointSymmetries createJointSymmetries(const std::string& symmetry_file)
{
    using namespace std;
    if (symmetry_file.empty())
        return JointSymmetries();
    ifstream is(symmetry_file);
    if (!is.is_open()) {
        cerr << "Could not open symmetry file: " << symmetry_file << endl;
        return JointSymmetries();
    }
    return JointSymmetries(is);
}

int main(int argc, char **argv)
{
    using namespace std;
    cout << "Hello SIRKA Calibration World" << endl;
    parse_command_line_options(argc, argv);

    // Open stream to write new joint sensor map. Calibration would be wasted if this fails.
    ofstream jsmostream(programOptions.calib_out + "/joint_sensor_map.txt");
    if (!jsmostream.is_open()) {
        cerr << "Could not open file to write calibrated joint-sensor-map to, so I'm not going to calculate it." << endl;
        return 1;
    }

    // Read joint sensor map.
    ifstream jsmstream(programOptions.jsm_file);
    if (!jsmstream.is_open()) {
        cerr << "Need joint-sensor assignment from the joint-sensor-map, although the vectors are subject to calibration, of course." << endl;
        return 1;
    }
    JointSensorMap jsm(jsmstream);

    // Read symmetries.
    const JointSymmetries joint_symmetries = createJointSymmetries(programOptions.symmetry_file);

    // Read IMU accumulates.
    vector<shared_ptr<LIR::IMUAccumulates>> imuAccumulates;
    imuAccumulates.reserve(programOptions.num_sensors);
    for (int i = 0; i < programOptions.num_sensors; ++i)
        imuAccumulates.push_back(shared_ptr<LIR::IMUAccumulates>(
                new LIR::IMUAccumulates(programOptions.imu_data_dir+"/sensor-"+to_string(i)+"-accumulate.log")));
    cout << "Loaded accumulates from " << imuAccumulates.size() << " IMUs. Number of readings:" << endl;
    list<LIR::IMUAccumulateEntry>::size_type minentries, maxentries;
    for (int i = 0; i < programOptions.num_sensors; ++i) {
        const list<LIR::IMUAccumulateEntry>::size_type size = imuAccumulates[i]->entries.size();
        cout << i << ": " << size << endl;
        if (!i || size > maxentries)
            maxentries = size;
        if (!i || size < minentries)
            minentries = size;
    }
    if (maxentries - minentries > 1) {
        cerr << "The numbers of entries in the accumulates per IMU differ by more than one. The accumulates appear not to be synchronized and are thus not suitable for calibration.";
        return 1;
    }

    /** Output data rate. The output data rate of the sensors is configured to be
        200Hz. This is not the frequency with which the recorder queries the sensor
        boards. Thus, the time differences between two samples can be substantially
        larger than 1/200 seconds. */
    const double odr = 200.0;
    struct StandardDeviations stddevs;
    stddevs.angular_velocity_density = 0.007 * M_PI/180.0; // In [rad/sqrt(Hz)]
    stddevs.angular_velocity = stddevs.angular_velocity_density * sqrt(odr);
    /* The acceleration density is so large to also hide calibration inaccuracies
       in the sensor noise. */
    stddevs.acceleration_density = 10.0 * 300e-6 * 9.81;     // In [m/s/s/sqrt(Hz)]
    stddevs.acceleration = stddevs.acceleration_density * sqrt(odr);
    stddevs.joint_velocity_difference = 0.1;
    stddevs.joint_velocity_difference_decorrelation_time = 0.1;
    stddevs.joint_axis_difference = 0.125*sqrt(2.0);
    stddevs.joint_axis_difference_decorrelation_time = 0.1;
    stddevs.velocity = 0.1;
    stddevs.velocity_decorrelation_time = 5; // 1; // 100.0;

    vector<TimedSensorStateRun> imuTrajectories;
    compute_initial_hinge_axes_guess(imuAccumulates, jsm, stddevs);
    calibrate_displacements(imuAccumulates, imuTrajectories, jsm, stddevs, joint_symmetries);

    jsm.save(jsmostream);
    for (int i = 0; i < programOptions.num_sensors; ++i) {
        /** The state of sensor i after 5 seconds. */
        const string filename = programOptions.calib_out + "/sensor-"+to_string(i)+"-5secondstate.log";
        saveOrientationAfterTime(imuTrajectories[i], 5.0, filename);
    }

    vector<Eigen::Quaterniond> sensorInWorld;
    sensorInWorld.reserve(programOptions.num_sensors);
    for (const TimedSensorStateRun& r : imuTrajectories) {
        shared_ptr<TimedSensorState> state = getStateAfterTime(r, 0.0);
        sensorInWorld.push_back(state->q_imuInWorld);
    }
    ofstream jsmasyplot(programOptions.prediction_out + "/jsm_plot.asy");
    cout << "Generating Asymptote program." << endl;
    jsm.exportToAsymptote(sensorInWorld, jsmasyplot);

    return 0;
}
