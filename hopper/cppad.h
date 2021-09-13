#ifndef IIT_ROBCOGEN_TPL_CPPAD_H
#define IIT_ROBCOGEN_TPL_CPPAD_H

#include <cppad/example/cppad_eigen.hpp> // for the Eigen traits

#include <iit/rbd/rbd.h>
#include <iit/rbd/scalar_traits.h>
#include <iit/robcogen/macros.h>

namespace CppAD {
    namespace cg {
        template<typename Base>
        struct ADCG : public AD<CG<Base>> {
            ADCG() = default;
            ADCG(const AD<CG<Base>>& ad) : AD<CG<Base>>(ad) {}
            ADCG(const CG<Base>& cg) : AD<CG<Base>>(cg) {}
            ADCG(const Base& b) : AD<CG<Base>>(b) {}
        };
    }

    template<typename Base>
    struct ok_if_S_same_as_T<AD<cg::CG<Base>>, cg::ADCG<Base>> {
        AD<cg::CG<Base>> value;
    };
}

namespace iit {
    namespace robcogen {
        namespace cppad {
            typedef CppAD::AD<double> Double;
            typedef CppAD::AD<float> Float;

            template<typename Scalar>
            struct TraitFuncs {
                inline static Scalar exp(const Scalar& x) { return CppAD::exp(x); }
                inline static Scalar sin(const Scalar& x) { return CppAD::sin(x); }
                inline static Scalar cos(const Scalar& x) { return CppAD::cos(x); }
                inline static Scalar tanh(const Scalar& x) { return CppAD::tanh(x); }
                inline static Scalar sqrt(const Scalar& x) { return CppAD::sqrt(x); }
                inline static Scalar abs(const Scalar& x) { return CppAD::abs(x); }

                inline static Scalar min(const Scalar& x, const Scalar& y) { return CppAD::CondExpLt(x, y, x, y); }
                inline static Scalar max(const Scalar& x, const Scalar& y) { return CppAD::CondExpGt(x, y, x, y); }
            };
        } //namespace 'cppad'

        template<typename CppADFloat>
        struct BasicTraits {
            typedef typename CppADFloat::value_type ValueType;
            typedef CppADFloat Scalar;
        };

        template<typename CppADFloat>
        struct ScalarTraits : public rbd::ScalarTraitsCommons<BasicTraits<CppADFloat> >,
                              public cppad::TraitFuncs<typename BasicTraits<CppADFloat>::Scalar> {
        };

        typedef ScalarTraits<cppad::Double> CppADDoubleTraits;
        typedef ScalarTraits<cppad::Float> CppADFloatTraits;
    } // namespace robcogen

    namespace rbd {
        template<>
        struct ScalarTraits<robcogen::cppad::Float> : public robcogen::CppADFloatTraits {
        };
        template<>
        struct ScalarTraits<robcogen::cppad::Double> : public robcogen::CppADDoubleTraits {
        };
    }

} // namespace iit


namespace Eigen {

// See here: https://stackoverflow.com/q/65019299
    template<typename Derived>
    iit::robcogen::cppad::Double
    operator+(const iit::rbd::MatrixBase<Derived>& lhs, const iit::robcogen::cppad::Double& rhs) {
        iit_rbd_matrix_specific_size(iit::rbd::MatrixBase<Derived>, 1, 1)
        return lhs.derived().coeff(0, 0) + rhs;
    }

    template<typename Derived, typename Base>
    CppAD::cg::ADCG<Base>
    operator+(const iit::rbd::MatrixBase<Derived>& lhs, const CppAD::cg::ADCG<Base>& rhs) {
        iit_rbd_matrix_specific_size(iit::rbd::MatrixBase<Derived>, 1, 1)
        return lhs.derived().coeff(0, 0) + rhs;
    }

}


#endif

