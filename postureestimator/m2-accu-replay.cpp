/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#include <githash.h>
#include <thread>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <jsmviz/jsmviz.h>
#include <Options/options.h>

#include <OrientationFromMotion/joint_sensor_map.h>
#include <OrientationFromMotion/orientation_from_motion.h>

#include <array>
#include <IMUAccumulate.h>

/**
 * Load the initial orientation for sensor with id 'sid' from the initial
 * orientation directory 'orient_dir'. The result is returned in 'orient'.
 * If something goes wrong, a string is thrown.
 */
void load_initial_orientation(Eigen::Matrix3f& orient, const std::string& orient_dir, const unsigned sid)
{
    const std::string filename = orient_dir + "/sensor-" + std::to_string(sid)
                                            + "-5secondstate.log";
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        BOOST_LOG_TRIVIAL(fatal) << "Could not open initial orientation file: " << filename;
        throw "Could not open initial orientation file.";
    }
    std::string line;
    std::getline(ifs, line);
    if (line[0] != '#' && line[0] != 'S') {
        BOOST_LOG_TRIVIAL(fatal) << "Error reading " << filename
                                 << ": Initial orientation file must start with a comment.";
        throw "Initial orientation file not formatted properly.";
    }
    std::getline(ifs, line);
    std::istringstream iss(line);
    Eigen::Quaternionf quat;
    float time;
    iss >> time;
    iss >> quat.w();
    iss >> quat.x();
    iss >> quat.y();
    iss >> quat.z();
    ifs.close();
    orient = quat.toRotationMatrix();
}

template <unsigned long n>
static void load_accumulates(const std::string& replay_path, std::array<std::shared_ptr<LIR::IMUAccumulates>, n>& imu_accumulates)
{
    BOOST_LOG_TRIVIAL(info) << "Loading accumulates from: " << replay_path;
    for (unsigned k=0; k<n; ++k) {
        const std::string filename = replay_path+"/sensor-"+std::to_string(k)+"-accumulate.log";
        BOOST_LOG_TRIVIAL(trace) << "Loading " << filename;
        imu_accumulates[k] = std::make_shared<LIR::IMUAccumulates>(filename);
    }
}

typedef std::list<LIR::IMUAccumulateEntry>::iterator AccuEntryPtr;
template <unsigned long n>
static bool increment_all(std::array<AccuEntryPtr, n>& entry,
                          std::array<AccuEntryPtr, n>& next,
                          const std::array<AccuEntryPtr, n>& end)
{
    for (unsigned k=0; k<n; ++k) {
        entry[k] = next[k]++;
        if (next[k] == end[k]) {
            return false;
        }
    }
    return true;
}

template <unsigned long n>
static unsigned max_delta_usecs(const std::array<AccuEntryPtr, n>& entry,
                                const std::array<AccuEntryPtr, n>& next)
{
    unsigned max = 0;
    for (unsigned k=0; k<n; ++k) {
        const unsigned delta = next[k]->usecs - entry[k]->usecs;
        if (delta>max)
            max = delta;
    }
    return max;
}

template <unsigned long n>
static bool increment_trailing(std::array<AccuEntryPtr, n>& entry,
                               std::array<AccuEntryPtr, n>& next,
                               const std::array<AccuEntryPtr, n>& end,
                               const unsigned max,
                               const unsigned tolerance)
{
    for (unsigned k=0; k<n; ++k) {
        while (next[k]->usecs - entry[k]->usecs <= max - tolerance) {
            ++next[k];
            if (next[k] == end[k]) {
                return false;
            }
        }
    }
    return true;
}


template <unsigned long n>
static void synchronize_accumulates(std::array<std::shared_ptr<LIR::IMUAccumulates>, n>& imu_accumulates,
                                    std::list<std::array<LIR::IMUAccumulateEntry *, n>>& resps)
{
    resps.clear();
    std::array<AccuEntryPtr, n> entry; /* Last used accumulate entry. */
    std::array<AccuEntryPtr, n> next; /* Next accumulate entry to compute delta accumulate to. */
    std::array<AccuEntryPtr, n> end; /* Terminal iterators for each accumulate series. */

    /* Initialize */
    {
        std::array<LIR::IMUAccumulateEntry *, n> resp;
        for (unsigned k=0; k<n; ++k) {
            end[k] = imu_accumulates[k]->entries.end();
            next[k] = imu_accumulates[k]->entries.begin();
            entry[k] = next[k];
            resp[k] = &(*next[k]);
        }
        resps.push_back(resp);
    }

    /* Find synchronized accumulates. */
    while (increment_all(entry, next, end)) {
        const unsigned max = max_delta_usecs(entry, next);
        if (!increment_trailing(entry, next, end, max, 50000))
            break;

        std::array<LIR::IMUAccumulateEntry *, n> resp;
        for (unsigned k=0; k<n; ++k) {
            resp[k] = &(*next[k]);
        }
        resps.push_back(resp);
    }
}

template <unsigned long n>
struct AccumulateAngVelPair
{
    Accumulate as[n];
    Vector3f ws[n];
};

template <unsigned long n>
void compute_delta_accumulates(std::list<AccumulateAngVelPair<n>>& delta_accumulates,
                               const std::list<std::array<LIR::IMUAccumulateEntry *, n>>& resps)
{
    typename std::list<std::array<LIR::IMUAccumulateEntry *, n>>::const_iterator end = resps.end();
    typename std::list<std::array<LIR::IMUAccumulateEntry *, n>>::const_iterator header = resps.begin();
    typename std::list<std::array<LIR::IMUAccumulateEntry *, n>>::const_iterator trailer = header++;

    for (; header != end; trailer = header++) {
        AccumulateAngVelPair<n> aavp;
        for (unsigned k=0; k<n; ++k) {
            const LIR::IMUAccumulateEntry& he = *((*header)[k]); /* heading accumulate entry */
            const LIR::IMUAccumulateEntry& te = *((*trailer)[k]); /* trailing accumulate entry */
            const Accumulate h(he.orientation.toRotationMatrix().cast<float>(), he.velocity.cast<float>(), he.usecs);
            const Accumulate t(te.orientation.toRotationMatrix().cast<float>(), te.velocity.cast<float>(), te.usecs);
            aavp.as[k] = t % h;
            aavp.ws[k] = he.angular_velocity.cast<float>();
        }
        delta_accumulates.push_back(aavp);
    }
}

template <unsigned long n>
static void load_initial_states(std::array<SensorState, n>& initial_states, std::array<MatrixXf, n>& initial_covars,
                                const std::string& orientations_dir)
{
    /* Create a prototype of the covariance matrices of the initial state.
       Currently, all initial covariances are the same. */
    MatrixXf proto_covar = MatrixXf::Zero(SensorState::DOF, SensorState::DOF);
    proto_covar.block<3,3>(0,0) = Matrix3f::Identity() * (5.0f/180.0f*M_PI)*(5.0f/180.0f*M_PI);
    proto_covar.block<3,3>(3,3) = Matrix3f::Identity();
    proto_covar.block<3,3>(6,6) = Matrix3f::Identity() * (0.014f * M_PI / 180.0f)*(0.014f * M_PI / 180.0f);

    for (unsigned k=0; k<n; ++k) {
        /* Load initial orientation and create initial state. */
        Matrix3f Q;
        load_initial_orientation(Q, orientations_dir, k);
        initial_states[k] = SensorState(Q, Vector3f::Zero(), Vector3f::Zero());

        /* Assemble covariance matrix of initial state. */
        initial_covars[k] = proto_covar;
    }
}

template<unsigned long n>
void run_ofm(OrientationFromMotion *ofm, std::list<AccumulateAngVelPair<n>> *imu_data, std::array<std::shared_ptr<SharedOrientationf>, n> sso,
             bool waitforwallclock, bool *running)
{
    typedef std::chrono::time_point<std::chrono::steady_clock> TimePoint;
    typedef std::chrono::duration<float> Duration;
    typename std::list<AccumulateAngVelPair<n>>::const_iterator it = imu_data->begin();
    const typename std::list<AccumulateAngVelPair<n>>::const_iterator end = imu_data->end();

    const TimePoint start = std::chrono::steady_clock::now();
    float time = 0.0;
    for (; it != end && *running; ++it) {
        const float deltat = it->as[0].durationSeconds();
        ofm->dynamic_update(it->as);
        ofm->measurement_update(it->ws, deltat);
        time += deltat;
        ofm->log(time);

        const Duration wallclock = std::chrono::steady_clock::now() - start;
        const Duration skew = Duration(time) - wallclock;
        if (waitforwallclock && skew > Duration::zero()) {
            std::this_thread::sleep_for(skew);
        }

        for (unsigned k=0; k<n; ++k) {
            sso[k]->set_data(ofm->get_orientation(k), time);
        }
    }
}

int main(int argc, char **argv)
{
    /* Greet the user. */
    BOOST_LOG_TRIVIAL(info) << "Hello, I'm the SIRKA M2 sensor fusion replayer.";
    BOOST_LOG_TRIVIAL(info) << "Project file for this binary was generated from git hash: " << Version::GITHASH;

    // MARK: Options
    /* Parse the command line options. */
    SpIntOption realtime(new IntOption(false, "realtime", "1: Don't replay data faster than wallclock time. 0: Replay as fast as possible."));
    std::list<SpIntOption> iopts{realtime};
    SpStringOption log_path(new StringOption(false, "logs", "Path where to store log files."));
    SpStringOption calibration_path(new StringOption(true, "calibrations", "Path to sensor node calibrations."));
    SpStringOption replay_path(new StringOption(true, "replay", "Path to the sensor log files to be replayed."));
    SpStringOption initial_orient(new StringOption(false, "initial_orientations", "Path where to load the orientation initialization from."));
    std::list<SpStringOption> sopts{calibration_path, log_path, replay_path, initial_orient};

    std::unique_ptr<CommonOptions> options;
    try {
        options.reset(new CommonOptions(argv, argc, iopts, sopts));
    } catch (const OptionError& error) {
        BOOST_LOG_TRIVIAL(fatal) << "Error in command line options: " << error.message;
        BOOST_LOG_TRIVIAL(info) << error.help;
        return 1;
    } catch (const std::string& help) {
        BOOST_LOG_TRIVIAL(info) << help;
        return 0;
    }

    BOOST_LOG_TRIVIAL(info) << "Will write logs into: " << log_path->value;
    BOOST_LOG_TRIVIAL(info) << "Will load calibrations from: " << calibration_path->value;

    // MARK: Set log level
    if (!options->verbose)
        boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::info);

    constexpr unsigned num_sensors = 15;
    std::array<std::shared_ptr<LIR::IMUAccumulates>, num_sensors> imu_accumulates;
    load_accumulates(replay_path->value, imu_accumulates);
    std::list<std::array<LIR::IMUAccumulateEntry *, num_sensors>> resps;
    synchronize_accumulates(imu_accumulates, resps);
    std::list<AccumulateAngVelPair<num_sensors>> delta_accumulates;
    compute_delta_accumulates(delta_accumulates, resps);

    /* Load joint sensor map from calibrations. */
    std::ifstream jsmfile(calibration_path->value + "/joint_sensor_map.txt");
    if (!jsmfile.is_open()) {
        BOOST_LOG_TRIVIAL(fatal) << "Can't load joint sensor map. Error: " << strerror(errno);
        return 1;
    }
    JointSensorMap jsm(jsmfile);
    jsmfile.close();
    assert (jsm.numJoints == 14);

    /* Define variance for the estimator. */
    const Variances variances = {
        .acceleration = (300e-6f * 9.81f * 300e-6f * 9.81f) * 200.0f /**< Bosch documentation */,
        .acceleration_density = (300e-6f * 9.81f * 300e-6f * 9.81f) * 100.0f,
        .angular_velocity = (0.007f * M_PI/180.0f)*(0.007f * M_PI/180.0f) * 200.0f,
        .angular_velocity_density = (0.007f * M_PI/180.0f)*(0.007f * M_PI/180.0f),
        .gyro_bias = (1.0f/180.0f*M_PI)*(1.0f/180.0f*M_PI) / 3600.0f, /**< 1 deg per hour gyro bias change. */
        .joint_velocity_difference_variance = 0.01f,
        .joint_velocity_difference_decorrelation_time = 0.1f,
        .velocity_variance = 0.1f,
        .velocity_decorrelation_time = 5.0f,
        .z_variance = 1.0f,
        .z_decorrelation_time = 300.0f,
        .hinge_axis_variance = 0.125f*0.125f,
        .hinge_axis_decorrelation_time = 0.1f
    };

    /* Set up the estimator. */
    OrientationFromMotion ofm(num_sensors, jsm, variances);
    ofm.set_log_path(log_path->value);

    /* Load initial sensor states and covarianes. */
    std::array<SensorState, num_sensors> initial_states;
    std::array<MatrixXf, num_sensors> initial_covars;
    if (initial_orient->set) {
        load_initial_states(initial_states, initial_covars, initial_orient->value);
        ofm.initialize(initial_states.data(), initial_covars.data());
    }

    /* Shared orientations to be used by the visulization. */
    std::array<std::shared_ptr<SharedOrientationf>, num_sensors> sso;
    for (int i = 0; i < num_sensors; ++i)
        sso[i] = std::make_shared<SharedOrientationf>(Eigen::Matrix3f::Identity());

    bool running = true;
    bool waitforwallclock = realtime->set && realtime->value == 1;
    std::thread ofm_thread(run_ofm<num_sensors>, &ofm, &delta_accumulates, sso, waitforwallclock, &running);
    /* Start the visualization. */
    jsmviz(jsm, sso);
    running = false;
    ofm_thread.join();

    return 0;
}
