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

#include <string>
#include <iostream>

#include <eigen3/Eigen/Eigen>

#include <shadowClient.hpp>
#include <shadowFormat.hpp>

// Defaults to "127.0.0.1"
const std::string Host = "";
// Use 32079 for preview data service.
// Use 32078 for sensor data service
// Use 32077 for raw data service.
// Use 32076 for configurable data service.
// Use 32075 for console service.
const unsigned PortPreview = 32079;
const unsigned PortSensor = 32078;
const unsigned PortRaw = 32077;
const unsigned PortConfigurable = 32076;
const unsigned PortConsole = 32075;

using namespace std;
using namespace Motion::SDK;


double oflow(double x) {
    if (x >= 50) {
        return x - 100;
    }
    if (x < -50 && x != fmod(x,100)) {
        return x + 100;
    }
    return x;
}

Eigen::Vector3d oflow(Eigen::Vector3d v) {
    v(0) = oflow(v(0));
    v(1) = oflow(v(1));
    v(2) = oflow(v(2));
}

int accu_init() {

}

int accu_addto(Eigen::Vector3d acc, Eigen::Vector3d gyro) {

}

int accu_finish() {

}

int read_raw (const std::string &host, const unsigned &port) {
    try {
        // Open connection to the data server.
        Client client(host, port);
        std::cout << "Connected to " << host << ":" << port << std::endl;

        Client::data_type data;
        while (client.readData(data)) {
            typedef Format::raw_service_type map_type;
            map_type raw = Format::Raw(data.begin(), data.end());
            
            if (!raw.empty()) {
                std::cout << Motion::SDK::Format::RawElement::Name << ": "<< raw.size();

                for (map_type::iterator itr = raw.begin(); itr != raw.end(); ++itr) {
                    Format::RawElement::data_type acc = itr->second.getAccelerometer();
                    Format::RawElement::data_type gyr = itr->second.getGyroscope();
                    
                    accu_addto( Eigen::Vector3d(acc[0], acc[1], acc[2]), 
                                Eigen::Vector3d(gyr[0], gyr[1], gyr[2]));
                }
            }
        }
    } catch(void* ex) {

    }
    return 0;
}


int main(int argc, char* argv[]) {
    read_raw("", PortRaw);
    return 0;
}