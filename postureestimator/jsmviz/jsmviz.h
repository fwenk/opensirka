/**
 * Copyright (c) 2017 Felix Wenk <felixwenk@googlemail.com>
 */

#ifndef __JSMVIZ_H_
#define __JSMVIZ_H_

#include <memory>
#include <Types/data_types.h>

#include <JointSensorMap.h>
typedef LIR::JointSensorMap<float> JointSensorMap;
typedef JointSensorMap::SensorLocation SensorLocation;

template <unsigned long n>
void jsmviz(const JointSensorMap& jsm, std::array<std::shared_ptr<SharedOrientationf>, n>& sso);

#endif // __JSMVIZ_H_
