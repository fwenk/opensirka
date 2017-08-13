#ifndef __ACCUMULATE_H
#define __ACCUMULATE_H

#include <Eigen/Dense>
#include <list>

namespace LIR {

template<typename real = double>
struct Accumulate
{
    enum { DOF = 6 };
    typedef Eigen::Quaternion<real>   Quaternion;
    typedef Eigen::Matrix<real, 3, 1> Vector3;

    Quaternion Q; /**< Accumulated orientation. (end frame in start frame) */
    Vector3 v; /**< Accumulated velocity, not gravity-compenstated, in start frame. */
    unsigned duration; /**< Accumulation duration in microseconds. */

    Accumulate(const Quaternion& Q, const Vector3& v, const real duration) : Q(Q), v(v), duration(static_cast<unsigned int>(duration * static_cast<real>(1e6))) {}
    Accumulate(const Quaternion& Q, const Vector3& v, const unsigned int duration) : Q(Q), v(v), duration(duration) {}
    Accumulate(const Accumulate& other) : Q(other.Q), v(other.v), duration(other.duration) {}
    Accumulate() : Q(Quaternion::Identity()), v(Vector3::Zero()), duration(0) {}

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
        const Quaternion Qinv = Q.inverse();
        return Accumulate(Qinv * rhs.Q, Qinv * overflow(rhs.v - v), rhs.duration - duration);
    }
    Accumulate& operator=(const Accumulate& rhs)
    {
        Q = rhs.Q;
        v = rhs.v;
        duration = rhs.duration;
        return *this;
    }
    real durationSeconds() const { return duration * static_cast<real>(1e-6); }
    static void overflow_inplace(Vector3& v)
    {
        constexpr real periodicity = 50.0;
        for (int d = 0; d < 3; ++d) {
            v(d) = std::fmod(v(d), static_cast<real>(2.0) * periodicity);
            if (v(d) >= periodicity)
                v(d) -= static_cast<real>(2.0) * periodicity;
            else if (v(d) < -periodicity)
                v(d) += static_cast<real>(2.0) * periodicity;
        }
    }
    static Vector3 overflow(const Vector3& v)
    {
        Vector3 ov(v);
        overflow_inplace(ov);
        return ov;
    }
};

template<typename real>
void compute_delta_accumulate_run(std::list<Accumulate<real>>& deltas, const std::list<Accumulate<real>>& accumulates)
{
    assert (accumulates.size() >= 2);
    typename std::list<Accumulate<real>>::const_iterator
        header = accumulates.begin(), trailer = header++;
    const typename std::list<Accumulate<real>>::const_iterator end = accumulates.end();

    deltas.clear();
    for (; header != end; ++header, ++trailer)
        deltas.push_back(*trailer % *header);
}

}

#endif
