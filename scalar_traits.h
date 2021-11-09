//
// Created by cryscan on 9/14/21.
//

#ifndef TO_IHC_2_SCALAR_TRAITS_H
#define TO_IHC_2_SCALAR_TRAITS_H

#include <type_traits>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cppad/cg.hpp>

template<typename Scalar>
struct Traits;

template<typename Base>
struct CppADCodeGenTraits {
    using CG = typename CppAD::cg::CG<Base>;
    using AD = typename CppAD::AD<CG>;

    struct Scalar : public AD {
        Scalar() = default;

        template<typename T, typename = std::enable_if_t<std::is_convertible_v<T, AD>>>
        Scalar(const T& t) : AD(t) {}

        Scalar(const Base& base) : AD(base) {}
    };

    using ValueType = Base;

    inline static Scalar exp(const Scalar& x) { return CppAD::exp(x); }
    inline static Scalar sin(const Scalar& x) { return CppAD::sin(x); }
    inline static Scalar cos(const Scalar& x) { return CppAD::cos(x); }
    inline static Scalar tanh(const Scalar& x) { return CppAD::tanh(x); }
    inline static Scalar sqrt(const Scalar& x) { return CppAD::sqrt(x); }
    inline static Scalar abs(const Scalar& x) { return CppAD::abs(x); }

    template<int Dims>
    inline static Eigen::Matrix<Scalar, Dims, 1>
    solve(const Eigen::Matrix<Scalar, Dims, Dims>& A, const Eigen::Matrix<Scalar, Dims, 1>& b) {
        auto LU = Traits<Scalar>::cholesky(A);
        return Traits<Scalar>::cholesky_solve(LU, b);
    }
};

#ifndef NDEBUG
template<typename Base>
struct CppAD::ok_if_S_same_as_T<CppAD::AD<CppAD::cg::CG<Base>>, typename CppADCodeGenTraits<Base>::Scalar> {
    CppAD::AD<CppAD::cg::CG<Base>> value;
};
#endif

template<typename Scalar>
struct Traits {
    // custom LU factorization
    // https://bitbucket.org/adrlab/hyq_gen_ad/src/master/include/external/iit/rbd/traits/CppADCodegenTrait.h
    template<int Dims, int Options = 0>
    inline static Eigen::Matrix<Scalar, Dims, Dims>
    cholesky(const Eigen::Matrix<Scalar, Dims, Dims, Options>& A) {
        Eigen::Matrix<Scalar, Dims, Dims> LU = Eigen::Matrix<Scalar, Dims, Dims>::Zero();

        for (int k = 0; k < Dims; ++k) {
            for (int i = k; i < Dims; ++i) {
                Scalar sum(0);
                for (int p = 0; p < k; ++p) sum += LU(i, p) * LU(p, k);
                LU(i, k) = A(i, k) - sum;
            }
            for (int j = k + 1; j < Dims; ++j) {
                Scalar sum(0);
                for (int p = 0; p < k; ++p) sum += LU(k, p) * LU(p, j);
                LU(k, j) = (A(k, j) - sum) / LU(k, k);
            }
        }

        return LU;
    }

    template<int Dims, int Options = 0>
    inline static Eigen::Matrix<Scalar, Dims, 1>
    cholesky_solve(const Eigen::Matrix<Scalar, Dims, Dims>& LU, const Eigen::Matrix<Scalar, Dims, 1, Options>& b) {
        Eigen::Matrix<Scalar, Dims, 1> x = Eigen::Matrix<Scalar, Dims, 1>::Zero();
        Eigen::Matrix<Scalar, Dims, 1> y = Eigen::Matrix<Scalar, Dims, 1>::Zero();

        for (int i = 0; i < Dims; ++i) {
            Scalar sum(0);
            for (int k = 0; k < i; ++k) sum += LU(i, k) * y(k);
            y(i) = (b(i) - sum) / LU(i, i);
        }
        for (int i = Dims - 1; i >= 0; --i) {
            Scalar sum(0);
            for (int k = i + 1; k < Dims; ++k) sum += LU(i, k) * x(k);
            x(i) = y(i) - sum;
        }

        return x;
    }

    template<int Dims, int N, int Options = 0>
    inline static Eigen::Matrix<Scalar, Dims, N>
    cholesky_solve(const Eigen::Matrix<Scalar, Dims, Dims>& LU, const Eigen::Matrix<Scalar, Dims, N, Options>& B) {
        Eigen::Matrix<Scalar, Dims, N> X;
        for (int i = 0; i < N; ++i) {
            Eigen::Matrix<Scalar, Dims, 1> col = B.col(i);
            X.col(i) << cholesky_solve(LU, col);
        }
        return X;
    }

    template<int Dims, int Options = 0>
    inline static Eigen::Matrix<Scalar, Dims, Dims> inverse(const Eigen::Matrix<Scalar, Dims, Dims, Options>& A) {
        return A.inverse();
    }

    inline static Eigen::Quaternion<Scalar> inverse(const Eigen::Quaternion<Scalar>& r) {
        if constexpr (std::is_same_v<Scalar, typename CppADCodeGenTraits<double>::Scalar>) {
            Scalar n2 = r.squaredNorm();
            n2 = CppAD::CondExpGt(n2, Scalar(0), n2, Scalar(1));
            return Eigen::Quaternion<Scalar>(r.conjugate().coeffs() / n2);
        } else return r.inverse();
    }

    template<int N, int Options = 0>
    inline static Eigen::Matrix<Scalar, N, 1> normalize(const Eigen::Matrix<Scalar, N, 1, Options>& vector) {
        if constexpr (std::is_same_v<Scalar, typename CppADCodeGenTraits<double>::Scalar>) {
            Scalar z = vector.norm();
            z = CppAD::CondExpGt(z, Scalar(0), z, Scalar(1));
            return vector / z;
        } else return vector.normalized();
    }
};

#endif //TO_IHC_2_SCALAR_TRAITS_H
