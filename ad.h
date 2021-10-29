//
// Created by cryscan on 10/28/21.
//

#ifndef TO_IHC_2_AD_H
#define TO_IHC_2_AD_H

#include <cppad/cg.hpp>

enum class EvalOption {
    ZERO_ORDER,
    FIRST_ORDER,
};

template<typename T>
struct Parameter {
};

template<typename Derived, typename ScalarTraits_, int InputDims, int ParamDims, int OutputDims>
class ADBase {
public:
    using ScalarTraits = ScalarTraits_;
    using Scalar = typename ScalarTraits::Scalar;

    using Params = Parameter<Derived>;

    static constexpr int input_dims = InputDims;
    static constexpr int param_dims = ParamDims;
    static constexpr int output_dims = OutputDims;

    using CG = CppAD::cg::CG<typename ScalarTraits::ValueType>;
    using AD = CppAD::AD<CG>;

    using ScalarVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using ADVector = Eigen::Matrix<AD, Eigen::Dynamic, 1>;

    CppAD::ADFun<CG> ad_fun[2];

    std::unique_ptr<CppAD::cg::DynamicLib<double>> lib;
    std::unique_ptr<CppAD::cg::GenericModel<double>> models[2];

    Params params;

protected:
    const std::string name;
    const std::string jacobian_name;
    const std::string library_name;


};

#endif //TO_IHC_2_AD_H
