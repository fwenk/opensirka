/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __IMUReading_H_
#define __IMUReading_H_

#include <list>
#include <Eigen/Dense>

namespace LIR {
    // Weird structure for compatibility with old calibrator implementation.
    struct Resolutions {
        float gyroscope;
        float accelerometer;
    };
    struct Resolutions load_resolutions(const std::string& file);
    void save_resolutions(const std::string& filename, const struct Resolutions& resolutions);

    struct IMUReadingEntry
    {
        Eigen::Vector3d gyro;
        Eigen::Vector3d accel;
        double time;
        long frame;
    };

    class IMUReading
    {
    public:
        std::list<IMUReadingEntry> reading;

        /** Constructor to read files with already scaled data. */
        IMUReading(const std::string& file);
        /** Constructor to read raw data files. */
        IMUReading(const std::string& file, const struct Resolutions& resolutions);
    };
}

#endif // __IMUReading_H_
