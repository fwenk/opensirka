/*
 * This code is licensed under the LGPL.
 * See LICENSE in the main directory of opensirka for details.
 */
#include "sphere.h"

#include <boost/math/special_functions/sinc.hpp>

/**
 * Compute the rotation matrix R mapping [1;0;0] to v,
 * i.e. v = R*[1;0;0]
 */
Eigen::Matrix3f Sphere::rotx(const Eigen::Vector3f& v)
{
    const float alpha = std::atan2(v.z(), v.y());
    const float r     = Eigen::numext::hypot(v.y(), v.z()); //sqrt(v.y()*v.y() + v.z()*v.z());
    const float c     = std::cos(alpha);
    const float s     = std::sin(alpha);
    return (Eigen::Matrix3f() << v.x(), -r, 0.0f,
                                 v.y(), v.x()*c, -s,
                                 v.z(), v.x()*s, c).finished();
}

/**
 * Boxplus operation on r\inS2, represented as a 3-dimensional unit vector.
 * r is modified in place, i.e. this function implements: r = r \bplus delta
 */
void Sphere::bplus_inplace(Eigen::Vector3f &r, const Eigen::Vector2f &delta)
{
    const float delta_norm = delta.norm();
    r = rotx(r) * (Eigen::Vector3f() << std::cos(delta_norm),
                                        boost::math::sinc_pi(delta_norm) * delta).finished();
}

/**
 * Boxplus operation on r \in S2, represented as a 3-dimensional vector.
 * This function implements s = r \bplus delta
 */
Eigen::Vector3f Sphere::bplus(const Eigen::Vector3f &r, const Eigen::Vector2f &delta)
{
    Eigen::Vector3f res(r);
    bplus_inplace(res, delta);
    return res;
}

/**
 * The derivative of (r \bplus delta) around delta = 0.
 */
Eigen::Matrix<float, 3, 2> Sphere::dotbplus(const Eigen::Vector3f &r)
{
    return rotx(r).rightCols<2>();
}
