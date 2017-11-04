/**
 * Authors:
 * Felix Wenk <felixwenk@googlemail.com>
 */
#define BOOST_TEST_MODULE "test_packed_matrix"

#include <boost/test/unit_test.hpp>
#include "../src/packed_triangular_matrix.h"

#ifdef __APPLE__
#include <Accelerate/Accelerate.h>
#else
#include <lapacke/lapacke.h>
#endif

static Eigen::Matrix<double, 6, 6> create_random_symmetric_matrix()
{
    Eigen::Matrix<double, 6, 6> X;
    X.setRandom();
    return X.transpose() * X;
}

BOOST_AUTO_TEST_CASE(test_indexing)
{
    const Eigen::Matrix<double, 6, 6> A = create_random_symmetric_matrix();
    PackedTriangularMatrix<6> A_packed;
    A_packed.setFromEigenMatrix(A);
    for (unsigned r = 0; r < 6; ++r)
        for (unsigned c = 0; c < 6; ++c)
            BOOST_CHECK_EQUAL(A(r, c), A_packed(r, c));
}

BOOST_AUTO_TEST_CASE(test_cholesky_decomposition_and_inverse)
{
    /* Less an actual test, this is more an exercise to play and learn how
       lapack functions are called. */
    const Eigen::Matrix<double, 6, 6> A = create_random_symmetric_matrix();
    PackedTriangularMatrix<6> A_packed;
    A_packed.setFromEigenMatrix(A);

    /* Compute the Cholesky factorization. */
    char uplo = 'L';
    int n = 6;
    int info = 0;
    dpptrf_(&uplo, &n, A_packed.elems, &info);

    Eigen::Matrix<double, 6, 6> L = A.llt().matrixL();
    for (unsigned r = 0; r < 6; ++r)
        for (unsigned c = 0; c <= r; ++c)
            BOOST_CHECK_CLOSE(L(r, c), A_packed(r, c), 0.01);

    /* Compute inverse of lower triangular matrix. */
    char diag = 'N';
    dtptri_(&uplo, &diag, &n, A_packed.elems, &info);
    Eigen::Matrix<double, 6, 6> Linv = L.inverse();
    for (unsigned r = 0; r < 6; ++r)
        for (unsigned c = 0; c <= r; ++c)
            BOOST_CHECK_CLOSE(Linv(r, c), A_packed(r, c), 0.01);
}

BOOST_AUTO_TEST_CASE(test_matrix_vector_multiplication)
{
    const Eigen::Matrix<double, 6, 6> A = create_random_symmetric_matrix();
    PackedTriangularMatrix<6> A_packed;
    A_packed.setFromEigenMatrix(A);
    /* Test vector multiplication with packed matrix (without LAPACK, because it also has to work with the Jet type. */
    Eigen::Matrix<double, 6, 1> v(Eigen::Matrix<double, 6, 1>::Random());
    Eigen::Matrix<double, 6, 1> Av(Eigen::Matrix<double, 6, 1>::Zero());
    Eigen::Matrix<double, 6, 1> Av_fast(Av);

    const Eigen::Matrix<double, 6, 1> Av_test = A_packed.toEigenMatrix() * v;
    lower_triangular_packed_matrix_mult<double, 6>(A_packed.elems, v.data(), Av_fast.data());
    for (unsigned k = 0; k < 6; ++k)
        BOOST_CHECK_CLOSE(Av_test(k), Av_fast(k), 0.01);
}
