/*
 * Authors:
 * Felix Wenk <felixwenk@googlemail.com>
 */

#define BOOST_TEST_MODULE "test_sphere_parameterization"

#include <boost/test/unit_test.hpp>
#include "../src/sphere_parameterization.h"

BOOST_AUTO_TEST_CASE(test_boxplus_magnitude)
{
    for (unsigned k = 0; k < 100; ++k) {
        Eigen::Vector3d v(Eigen::Vector3d::Random());
        v.normalize();
        Eigen::Vector2d delta(Eigen::Vector2d::Random());
        Eigen::Vector3d vplus;
        boxplus<double>(v.data(), delta.data(), vplus.data());
        BOOST_CHECK_CLOSE(vplus.norm(), 1.0, 0.0001);
    }
}

BOOST_AUTO_TEST_CASE(test_boxplus_boxminus)
{
    /* Random point on the unit sphere. */
    Eigen::Vector3d v(Eigen::Vector3d::Random());
    v.normalize();

    /* Random delta. */
    Eigen::Vector2d delta(Eigen::Vector2d::Random());

    Eigen::Vector3d v_plus;
    boxplus<double>(v.data(), delta.data(), v_plus.data());
    Eigen::Vector2d delta_test;
    boxminus<double>(v_plus.data(), v.data(), delta_test.data());
    BOOST_CHECK_CLOSE(delta(0), delta_test(0), 0.001);
    BOOST_CHECK_CLOSE(delta(1), delta_test(1), 0.001);

    Eigen::Vector3d v2(Eigen::Vector3d::Random());
    v2.normalize();
    Eigen::Vector2d delta2;
    boxminus<double>(v2.data(), v.data(), delta2.data());
    Eigen::Vector3d v2_test;
    boxplus<double>(v.data(), delta2.data(), v2_test.data());
    for (unsigned k = 0; k < 3; ++k)
        BOOST_CHECK_CLOSE(v2(k), v2_test(k), 0.001);

    BOOST_CHECK(true);
}
