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
};

template<typename Base>
struct CppAD::ok_if_S_same_as_T<CppAD::AD<CppAD::cg::CG<Base>>, typename CppADCodeGenTraits<Base>::Scalar> {
    CppAD::AD<CppAD::cg::CG<Base>> value;
};

#endif //TO_IHC_2_SCALAR_TRAITS_H
