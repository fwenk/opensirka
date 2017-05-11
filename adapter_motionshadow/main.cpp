/**
 *  MotionShadow Adapter for OpenSIRKA.
 *
 *  Copyright (C) 2017 Carl von Ossietzky University Oldenburg.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Christian Lins <christian.lins@uni-oldenburg.de>
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <queue>

#include <eigen3/Eigen/Eigen>
#include <boost/program_options.hpp>

#include <shadowClient.hpp>
#include <shadowFormat.hpp>

#include "../calibrator/src/Accumulate.h"
#include "../postureestimator/MathHelper/expm.h"

#define NUM_SENSORS 19

// Defaults to "127.0.0.1"
const std::string Host = "";
// Use 32079 for preview data service.
// Use 32078 for sensor data service
// Use 32077 for raw data service.
// Use 32076 for configurable data service.
// Use 32075 for console service.
//const unsigned PortPreview = 32079;
//const unsigned PortSensor = 32078;
const unsigned PortRaw = 32077;
//const unsigned PortConfigurable = 32076;
//const unsigned PortConsole = 32075;

using namespace std;
using namespace Motion::SDK;

Eigen::Quaterniond toQuaternion(Eigen::Vector3d euler) {
    Eigen::AngleAxisd rollAngle(euler(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(euler(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(euler(2), Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

/**
 *  Initializes accu und omega vectors.
 */
int accus_init(vector<struct Accumulate*>* accus, vector<Eigen::Vector3d>* omega) {
    for (int n = 0; n < NUM_SENSORS; n++) {
        struct Accumulate* accu = new struct Accumulate();
        accus->push_back(accu);
        if (omega != nullptr) {
            omega->push_back(Eigen::Vector3d(0,0,0));
        }
    }

    #ifdef DEBUG
    for (int n = 0; n < NUM_SENSORS; n++) {
        cout << (unsigned long)accus->operator[](n) << " ";
    }
    cout << endl;
    #endif

    return 0;
}

/**
 *  Creates an accumulate increment from sensor values and adds it
 *  to current accumulate.
 *  
 *  @param accu Pointer to current accumulate
 *  @param a Accelerometer value
 *  @param g Gyroscope value
 *  @param dt Time interval
 */
int accu_addto(int idx, struct Accumulate* accu, 
    Eigen::Vector3d a, Eigen::Vector3d w, unsigned int dt) 
{
    Eigen::Matrix3d R;
    Rot::expm(R, w * dt); // R is rotation matrix to rotate a in the originating coord system
    Eigen::Vector3d v = R * a * dt; // Integrate accelerometer to get velocity

    Eigen::Quaterniond Q = toQuaternion(w * dt);

    struct Accumulate update(Q, v, dt);
    *accu *= update;

    return 0;
}

int accus_clear(vector<struct Accumulate*>* accus) {
    std::for_each(accus->begin(), accus->end(), std::default_delete<struct Accumulate>());
    accus->clear();
    return 0;
}

int accus_finish(vector<struct Accumulate*>* accus, vector<ofstream*>* outputFiles, vector<Eigen::Vector3d>* omega) {
    auto now  = chrono::high_resolution_clock::now();
    auto usec = chrono::time_point_cast<chrono::microseconds>(now).time_since_epoch().count();

    for (int n = 0; n < NUM_SENSORS; n++) {
        const struct Accumulate* accu = (*accus)[n];
        *(*outputFiles)[n] << usec << " " << accu->Q.coeffs().transpose() << " "
                          << accu->v.transpose() << " "
                          << omega->operator[](n).transpose() << " "
                          << endl;
    }

    accus_clear(accus);
    accus_init(accus, nullptr);

    return 0;
}

int read_raw (const std::string &host, const unsigned &port, vector<ofstream*>& outputFiles) {
    Client client(host, port);
    std::cout << "Connected to " << host << ":" << port << std::endl;

    // It seems that the MotionShadow API does not provide timing
    // information, so we must calculate time durations by ourself.
    auto start = std::chrono::high_resolution_clock::now();
    auto sampling = 10;

    Client::data_type data;
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> rawbuf(NUM_SENSORS);
    vector<struct Accumulate*> accus;
    vector<Eigen::Vector3d> omega;

    accus_init(&accus, &omega);

    while (client.readData(data)) {
        // client.readData provides us with realtime samples, says the docs.
        typedef Format::raw_service_type map_type;
        map_type raw = Format::Raw(data.begin(), data.end());

        if (!raw.empty()) {
            // One raw element contains 19 samples, one for each sensor node
            //std::cout << Motion::SDK::Format::RawElement::Name << ": "<< raw.size() << endl;
            if (raw.size() != 19) {
                cout << "raw.size() != 19. Skip sample." << endl;
                continue;
            }

            int n = 0;
            for (map_type::iterator itr = raw.begin(); itr != raw.end(); ++itr) {
                Format::RawElement::data_type acc = itr->second.getAccelerometer();
                Format::RawElement::data_type gyr = itr->second.getGyroscope();
                
                omega[n] = Eigen::Vector3d(gyr[0], gyr[1], gyr[2]);
                rawbuf[n++] = make_pair(Eigen::Vector3d(acc[0], acc[1], acc[2]), 
                                        Eigen::Vector3d(gyr[0], gyr[1], gyr[2]));
            }
        }
        
        auto elapsed = chrono::high_resolution_clock::now() - start;
        auto duration = chrono::duration_cast<chrono::microseconds>(elapsed).count();
        start = std::chrono::high_resolution_clock::now();
        if (rawbuf.size() == 19) {
            for (int n = 0; n < 19; n++) {
                accu_addto(n, accus[n], rawbuf[n].first, rawbuf[n].second, (unsigned int)duration);
            }
        }

        if (--sampling == 0) {
            sampling = 10;
            accus_finish(&accus, &outputFiles, &omega);
        }
    }
    
    return 0;
}


int main(int argc, char* argv[]) {
    namespace po = boost::program_options;

    po::options_description desc("shadow command line options");
    desc.add_options()
    ("log-raw-data", po::value<std::string>(), "Logs the raw accelerometer and gyroscope data to files");

    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).run();
    po::store(parsed, vm);

    vector<ofstream*> outputFiles;
    for (int n = 0; n < NUM_SENSORS; n++) {
        stringstream name; name << "sensor-" << n << "-accumulate.log";
        outputFiles.push_back(new ofstream(name.str().c_str()));
    }

    read_raw("", PortRaw, outputFiles);
    return 0;
}