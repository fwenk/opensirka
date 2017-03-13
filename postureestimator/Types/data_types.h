/**
 * Copyright (c) 2015 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 */
#ifndef __DATA_TYPES_H_
#define __DATA_TYPES_H_

#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

/**
 * Class to communicate an orientation in a thread-save way.
 */
template <typename T>
class SharedThing
{
public:
  SharedThing(const T& data) : data(data), valid(false), time(-1.0) {}
  void set_data(const T& data, const double time = -1.0)
  {
    const boost::lock_guard<boost::mutex> guard(mutex);
    this->data = data;
    this->valid = true;
    this->time = time;
  }
  const T& get_data() const
  {
    const boost::lock_guard<boost::mutex> guard(mutex);
    return data;
  }
  void get_data(T& data, bool& valid) const
  {
    const boost::lock_guard<boost::mutex> guard(mutex);
    data = this->data;
    valid = this->valid;
  }
  const double get_time() const
  {
    const boost::lock_guard<boost::mutex> guard(mutex);
    return time;
  }
  bool is_valid() const
  {
    const boost::lock_guard<boost::mutex> guard(mutex);
    return valid;
  }
private:
  mutable boost::mutex mutex;
  T data;
  bool valid;
  double time;
};

typedef SharedThing<Eigen::Matrix3d> SharedOrientation;
typedef SharedThing<Eigen::Vector3d> SharedVector;
typedef SharedThing<Eigen::Matrix3f> SharedOrientationf;
typedef SharedThing<Eigen::Vector3f> SharedVectorf;

#endif // __DATA_TYPES_H_
