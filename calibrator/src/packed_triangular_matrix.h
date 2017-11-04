/**
 * Copyright (c) 2016 DFKI GmbH
 *
 * Author: Felix Wenk <felix.wenk@dfki.de>
 * Author: Felix Wenk <felixwenk@googlemail.com> (2017)
 */
#ifndef __PACKED_TRIANGULAR_MATRIX_H_
#define __PACKED_TRIANGULAR_MATRIX_H_

#include <Eigen/Dense>

template<unsigned N>
struct PackedTriangularMatrix {
    static constexpr unsigned numelems = (N*(N+1)/2);
    double elems[numelems];
    double operator()(unsigned row, unsigned col) const {
        if (col > row) {
            return operator()(col, row);
        } else {
            return elems[row + (2*N-1 - col) * col/2];
        }
    }
    Eigen::Matrix<double, N, N> toEigenMatrix(bool symmetric = false) const {
        Eigen::Matrix<double, N, N> A(Eigen::Matrix<double, 6, 6>::Zero());
        for (unsigned c = 0; c < N; ++c)
            for (unsigned r = (symmetric ? 0 : c); r < N; ++r)
                A(r, c) = operator()(r, c);
        return A;
    }
    void setFromEigenMatrix(const Eigen::Matrix<double, N, N>& A) {
        double *p = elems;
        for (unsigned c = 0; c < 6; ++c)
            for (unsigned r = c; r < 6; ++r)
                *p++ = A(r,c);
    }
};

template<typename T, unsigned dim>
static void lower_triangular_packed_matrix_mult(const double * const A, const T * const v, T *Av)
{
    const double *A_ptr = A;
    const T *v_ptr = v;
    const T *v_end = v + dim;
    for (T *res = Av, *res_end = Av + dim; res != res_end; ++res)
        *res = T(0.0);
    for (T *res_start = Av, *res_end = Av + dim;
         v_ptr != v_end; ++v_ptr, ++res_start) {
        for (T *res_ptr = res_start; res_ptr != res_end; ++res_ptr) {
            *res_ptr += T(*A_ptr++) * *v_ptr;
        }
    }
}

#endif
