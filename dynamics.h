//
// Created by cryscan on 8/17/21.
//

#ifndef TO_IHC_2_DYNAMICS_H
#define TO_IHC_2_DYNAMICS_H

#include <Eigen/Geometry>

#include "hopper/declarations.h"
#include "hopper/transforms.h"
#include "hopper/jacobians.h"
#include "hopper/inertia_properties.h"
#include "hopper/jsim.h"
#include "hopper/inverse_dynamics.h"

struct Dynamics {
    using Scalar = Hopper::rcg::Scalar;
    using ScalarTraits = Hopper::rcg::ScalarTraits;

    static constexpr int joint_space_dims = Hopper::rcg::JointSpaceDimension;
    static constexpr int state_dims = joint_space_dims + joint_space_dims;
    static constexpr int action_dims = 2;
    static constexpr int input_dims = state_dims + action_dims + 1;
    static constexpr int output_dims = state_dims + 3;

    using Vector3 = Hopper::rcg::Vector3;
    using Matrix3 = Hopper::rcg::Matrix<3, 3>;
    using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;

    using JointState = Hopper::rcg::JointState;
    using Control = Hopper::rcg::Matrix<action_dims, 1>;
    using ContactJacobian = Hopper::rcg::Matrix<3, joint_space_dims>;
    using ContactJacobianTranspose = Hopper::rcg::Matrix<joint_space_dims, 3>;

    using State = Eigen::Matrix<double, state_dims, 1>;
    using Action = Eigen::Matrix<double, action_dims, 1>;
    struct Params {
        State x;
        Action u;
        double active;
    };

    Dynamics(int num_iters, const Scalar& dt, const Scalar& mu)
            : inverse_dynamics(inertia_properties, motion_transforms),
              jsim(inertia_properties, force_transforms),
              num_iters(num_iters),
              dt(dt),
              mu(mu) {}

    void print(std::ostream& os, const Params& params) const;

    void build_map();
    void evaluate(const Params& params);

    auto get_f() const { return f; }
    auto get_foot_pos() const { return foot_pos; }
    auto get_df_dx() const { return df_dx; }
    auto get_df_du() const { return df_du; }

private:
    mutable Hopper::rcg::HomogeneousTransforms transforms;
    mutable Hopper::rcg::MotionTransforms motion_transforms;
    mutable Hopper::rcg::ForceTransforms force_transforms;

    mutable Hopper::rcg::Jacobians jacobians;
    mutable Hopper::rcg::InertiaProperties inertia_properties;
    mutable Hopper::rcg::InverseDynamics inverse_dynamics;
    mutable Hopper::rcg::JSIM jsim;

    const int num_iters;
    const Scalar dt;
    const Scalar mu;

    JointState q, u;
    Control tau;
    Scalar active;

    Hopper::rcg::Matrix<Eigen::Dynamic, 1> ad_x{input_dims};
    Hopper::rcg::Matrix<Eigen::Dynamic, 1> ad_y{output_dims};
    CppAD::ADFun<double> ad_fun;

    State f;
    Eigen::Vector3d foot_pos;
    Eigen::Matrix<double, state_dims, state_dims> df_dx;
    Eigen::Matrix<double, state_dims, action_dims> df_du;

    std::tuple<JointState, JointState> step() const;
    [[nodiscard]] Vector3 prox(const Vector3& p) const;

    Vector3 compute_foot_pos() const;
};


#endif //TO_IHC_2_DYNAMICS_H
