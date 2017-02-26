/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
#include "TimedSensorState.h"


#include <fstream>
#include <iostream>

using namespace std;

void TimedSensorState::save(ostream& ostream) const
{
    ostream << time << ' ' << q_imuInWorld.w() << ' ' << q_imuInWorld.x() << ' ' << q_imuInWorld.y() << ' ' << q_imuInWorld.z() << endl;
}

void saveTimedSensorStateRun(const TimedSensorStateRun& run, const string& filename)
{
    ofstream os(filename);
    if (!os.is_open()) {
        cerr << "Can not open output stream at: " << filename << endl;
        return;
    }
    os << "Sensor Orientation Trajectory as timed quaternions: seconds w x y z" << std::endl;
    for (const std::shared_ptr<TimedSensorState> tss : run)
        tss->save(os);
}

shared_ptr<TimedSensorState> getStateAfterTime(const TimedSensorStateRun& run, const double seconds)
{
    const TimedSensorStateRun::const_iterator
    state = std::lower_bound(run.begin(), run.end(), seconds,
                             [](const shared_ptr<TimedSensorState>& s, const double& t) -> bool { return s->time < t; });
    if (state == run.end()) {
        cerr << "Can not get " << seconds << "s state. Trajectory of sensor too short." << endl;
        throw 1;
    }
    return *state;
}

void saveOrientationAfterTime(const TimedSensorStateRun& run, const double seconds,
                              const string& filename)
{
    shared_ptr<TimedSensorState> state;
    try {
        state = getStateAfterTime(run, seconds);
    } catch (int e) {
        return;
    }

    ofstream os(filename);
    if (!os.is_open()) {
        cerr << "Could not open " << filename << endl;
        return;
    }
    os << "# Sensor Orientation after " << seconds << " seconds as a quaternion. Format: seconds w x y z" << std::endl;
    state->save(os);
}

