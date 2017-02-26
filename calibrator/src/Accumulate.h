#ifndef __ACCUMULATE_H
#define __ACCUMULATE_H

#include <Eigen/Dense>
#include <list>

struct Accumulate
{
    enum { DOF = 6 };
    Eigen::Quaterniond Q; /**< Accumulated orientation. (end frame in start frame) */
    Eigen::Vector3d v; /**< Accumulated velocity, not gravity-compenstated, in start frame. */
    unsigned duration; /**< Accumulation duration in microseconds. */

    Accumulate(const Eigen::Quaterniond& Q, const Eigen::Vector3d& v, const double duration) : Q(Q), v(v), duration(static_cast<unsigned int>(duration * 1e6)) {}
    Accumulate(const Eigen::Quaterniond& Q, const Eigen::Vector3d& v, const unsigned int duration) : Q(Q), v(v), duration(duration) {}
    Accumulate(const Accumulate& other) : Q(other.Q), v(other.v), duration(other.duration) {}
    Accumulate() : Q(Eigen::Quaterniond::Identity()), v(Eigen::Vector3d::Zero()), duration(0) {}

    Accumulate& operator*=(const Accumulate& rhs)
    {
        v += Q * rhs.v;
        overflow_inplace(v);
        Q *= rhs.Q;
        duration += rhs.duration;
        return *this;
    }
    Accumulate operator*(const Accumulate& rhs) const
    {
        Accumulate r(*this);
        r *= rhs;
        return r;
    }
    Accumulate operator%(const Accumulate& rhs) const
    {
        const Eigen::Quaterniond Qinv = Q.inverse();
        return Accumulate(Qinv * rhs.Q, Qinv * overflow(rhs.v - v), rhs.duration - duration);
    }
    Accumulate& operator=(const Accumulate& rhs)
    {
        Q = rhs.Q;
        v = rhs.v;
        duration = rhs.duration;
        return *this;
    }
    double durationSeconds() const { return duration * 1e-6; }
    static void overflow_inplace(Eigen::Vector3d& v)
    {
        const double periodicity = 50.0f;
        for (int d = 0; d < 3; ++d) {
            v(d) = std::fmod(v(d), 2.0f * periodicity);
            if (v(d) >= periodicity)
                v(d) -= 2.0f * periodicity;
            else if (v(d) < -periodicity)
                v(d) += 2.0f * periodicity;
        }
    }
    static Eigen::Vector3d overflow(const Eigen::Vector3d& v)
    {
        Eigen::Vector3d ov(v);
        overflow_inplace(ov);
        return ov;
    }
};

typedef std::list<Accumulate> AccumulateRun;
void compute_delta_accumulate_run(AccumulateRun& deltas, const AccumulateRun& accumulates);

#endif
