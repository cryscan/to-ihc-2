//
// Created by cryscan on 9/14/21.
//

#ifndef TO_IHC_2_SCALAR_TRAITS_H
#define TO_IHC_2_SCALAR_TRAITS_H

#include <type_traits>
#include <cppad/cg.hpp>

template<typename Base>
class CppADCodeGenTraits {
    typedef typename CppAD::cg::CG<Base> CG;
    typedef typename CppAD::AD<CG> AD;

public:
    struct Scalar : public AD {
        Scalar() = default;
        Scalar(const AD& ad) : AD(ad) {}
        Scalar(const CG& cg) : AD(cg) {}
        Scalar(const Base& b) : AD(b) {}
    };

    typedef typename Scalar::value_type ValueType;

    inline static Scalar exp(const Scalar& x) { return CppAD::exp(x); }
    inline static Scalar sin(const Scalar& x) { return CppAD::sin(x); }
    inline static Scalar cos(const Scalar& x) { return CppAD::cos(x); }
    inline static Scalar tanh(const Scalar& x) { return CppAD::tanh(x); }
    inline static Scalar sqrt(const Scalar& x) { return CppAD::sqrt(x); }
    inline static Scalar abs(const Scalar& x) { return CppAD::abs(x); }

    inline static Scalar min(const Scalar& x, const Scalar& y) { return CppAD::CondExpLt(x, y, x, y); }
    inline static Scalar max(const Scalar& x, const Scalar& y) { return CppAD::CondExpGt(x, y, x, y); }

    inline static Scalar remove_nan(const Scalar& x, const Scalar& y) {
        return CppAD::CondExpEq(x, x, x, y);
    }

    template<int Dims>
    inline static Eigen::Matrix<Scalar, Dims, 1>
    solve(const Eigen::Matrix<Scalar, Dims, Dims>& A, const Eigen::Matrix<Scalar, Dims, 1>& b) {
        auto LU = cholesky(A);
        return cholesky_solve(LU, b);
    }

    // custom LU factorization
    // https://bitbucket.org/adrlab/hyq_gen_ad/src/master/include/external/iit/rbd/traits/CppADCodegenTrait.h
    template<int Dims>
    static Eigen::Matrix<Scalar, Dims, Dims>
    cholesky(const Eigen::Matrix<Scalar, Dims, Dims>& A) {
//        for (int k = 0; k < S.rows(); ++k) {
//            for (int i = k; i < S.rows(); ++i) {
//                T sum(0.);
//                for (int p = 0; p < k; ++p)sum += D(i, p) * D(p, k);
//                D(i, k) = S(i, k) - sum; // not dividing by diagonals
//            }
//            for (int j = k + 1; j < S.rows(); ++j) {
//                T sum(0.);
//                for (int p = 0; p < k; ++p)sum += D(k, p) * D(p, j);
//                D(k, j) = (S(k, j) - sum) / D(k, k);
//            }
//        }

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

    template<int Dims>
    static Eigen::Matrix<Scalar, Dims, 1>
    cholesky_solve(const Eigen::Matrix<Scalar, Dims, Dims>& LU, const Eigen::Matrix<Scalar, Dims, 1>& b) {
        Eigen::Matrix<Scalar, Dims, 1> x = Eigen::Matrix<Scalar, Dims, 1>::Zero();
        Eigen::Matrix<Scalar, Dims, 1> y = Eigen::Matrix<Scalar, Dims, 1>::Zero();

//        const int d = rowAndCol;
//        T y[d];
//        for(int i=0;i<d;++i){
//            T sum(0.0);
//            for(int k=0;k<i;++k)sum+=LU(i,k)*y[k];
//            y[i]=(b(i)-sum)/LU(i,i);
//        }
//        for(int i=d-1;i>=0;--i){
//            T sum(0.);
//            for(int k=i+1;k<d;++k)sum+=LU(i,k)*x(k);
//            x(i)=(y[i]-sum); // not dividing by diagonals
//        }
//    }

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
};

template<typename Base>
struct CppAD::ok_if_S_same_as_T<CppAD::AD<CppAD::cg::CG<Base>>, typename CppADCodeGenTraits<Base>::Scalar> {
    CppAD::AD<CppAD::cg::CG<Base>> value;
};

#endif //TO_IHC_2_SCALAR_TRAITS_H
