//
// Created by cryscan on 8/18/21.
//

#ifndef TO_IHC_2_COMMON_H
#define TO_IHC_2_COMMON_H

#include "hopper/declarations.h"
#include "hopper/transforms.h"
#include "hopper/jacobians.h"
#include "hopper/inertia_properties.h"
#include "hopper/jsim.h"
#include "hopper/inverse_dynamics.h"

static constexpr int joint_space_dims = Hopper::rcg::JointSpaceDimension;
static constexpr int state_dims = joint_space_dims + joint_space_dims;
static constexpr int action_dims = 2;

using State = Eigen::Matrix<double, state_dims, 1>;
using Action = Eigen::Matrix<double, action_dims, 1>;

enum class EvalOption {
    ZERO_ORDER,
    FIRST_ORDER,
};

template<typename T, int InputDims, int OutputDims>
struct ADBase {
    using Params = T;

    using Scalar = Hopper::rcg::Scalar;
    using ScalarTraits = Hopper::rcg::ScalarTraits;
    using JointState = Hopper::rcg::JointState;
    using Action = Hopper::rcg::Matrix<action_dims, 1>;

    static constexpr int input_dims = InputDims;
    static constexpr int output_dims = OutputDims;

    ADBase() = default;
    ADBase(const ADBase& other) : params(other.params) {}
    ADBase& operator=(const ADBase& other) { params = other.params; }

    virtual void build_map() {};

    void evaluate(EvalOption option = EvalOption::FIRST_ORDER) { this->evaluate(params, option); }
    virtual void evaluate(const Params&, EvalOption option) {};

    static constexpr ADBase<Params, input_dims, output_dims> base_type() {}

    Params params;

protected:
    Hopper::rcg::Matrix<Eigen::Dynamic, 1> ad_x{input_dims};
    Hopper::rcg::Matrix<Eigen::Dynamic, 1> ad_y{output_dims};
    CppAD::ADFun<double> ad_fun;
};

#define ASSIGN_VECTOR(to, from, it, size) (to) = (from).segment<(size)>(it); (it) += (size);
#define ASSIGN_COLS(to, from, it, size) (to) = (from).middleCols<(size)>(it); (it) += (size);
#define FILL_VECTOR(to, from, it, size) (to).segment<(size)>(it) = (from); (it) += (size);

#endif //TO_IHC_2_COMMON_H
