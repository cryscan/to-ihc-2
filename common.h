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

struct Params {
    State x;
    Action u;
    double active;
};

template<int InputDims, int OutputDims>
struct ADBase {
    using Scalar = Hopper::rcg::Scalar;
    using ScalarTraits = Hopper::rcg::ScalarTraits;
    using JointState = Hopper::rcg::JointState;
    using Action = Hopper::rcg::Matrix<action_dims, 1>;

    static constexpr int input_dims = InputDims;
    static constexpr int output_dims = OutputDims;

    virtual void build_map() {};
    virtual void evaluate(const Params& params) {};

protected:
    Hopper::rcg::Matrix<Eigen::Dynamic, 1> ad_x{input_dims};
    Hopper::rcg::Matrix<Eigen::Dynamic, 1> ad_y{output_dims};
    CppAD::ADFun<double> ad_fun;
};

#endif //TO_IHC_2_COMMON_H
