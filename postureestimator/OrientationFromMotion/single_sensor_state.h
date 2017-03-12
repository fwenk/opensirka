/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __SINGLE_SENSOR_STATE_H_
#define __SINGLE_SENSOR_STATE_H_

#include "ofm_types.h"
#include <ostream>

class SensorState {
    Matrix3f Q; /**< Sensor orientation (sensorInWorld). */
    Vector3f v; /**< Sensor velocity (gravity compenstated, in world coordinates). */
    Vector3f b; /**< Gyro bias. */

public:
    enum { DOF = 9 };
    SensorState();
    SensorState(const Matrix3f& Q, const Vector3f& v, const Vector3f& b);
    SensorState& operator+=(const Vector9f& d);
    SensorState operator+(const Vector9f& d) const;
    Vector9f operator-(const SensorState& other) const;

    void single_dynamic_model(const Accumulate& a, Matrix9f& jacobianA, Matrix9f6& jacobianB);

    friend class OrientationFromMotion;

    friend std::ostream& operator<<(std::ostream& os, const SensorState& s);

    friend class AccuSensorPinboard; // TODO: Remove debug stuff.

    friend class SensorLogStream;
    // Some tests are friends.
    friend struct dynamic_update_bias_removal_test;
    friend struct measurement_update_test;
};

/** Computes s = s bplus d, s being an array of N sensor states. */
void arraybplus(SensorState * const s, const VectorXf& d, const int N);
/** Computes s2 = s1 bplus d, s2 and s1 being arrays of N sensor states. */
void arraybplus(SensorState * const s2, const SensorState * const s1, const VectorXf& d, const int N);
/** Computes d = s2 bminus s1, s2 and s1 being arrays of N sensor states. */
void arraybminus(VectorXf& d, const SensorState * const s2, const SensorState * const s1, const int N);

#endif // __SINGLE_SENSOR_STATE_H_
