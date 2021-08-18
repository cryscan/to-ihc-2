//
// Created by cryscan on 8/17/21.
//

#ifndef TO_IHC_2_DYNAMICS_H
#define TO_IHC_2_DYNAMICS_H

#include <Eigen/Geometry>

#include "common.h"

struct DynamicsParams {
    State x;
    Action u;
    double active;
};

#define INPUT_DIMS  state_dims + action_dims + 1
#define OUTPUT_DIMS state_dims + 3
#define BASE        ADBase<DynamicsParams, INPUT_DIMS, OUTPUT_DIMS>

struct Dynamics : public BASE {
    using Base = BASE;
    using Base::Params;

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::JointState;
    using Base::Action;

    using Base::input_dims;
    using Base::output_dims;

    using Vector3 = Hopper::rcg::Vector3;
    using Matrix3 = Hopper::rcg::Matrix<3, 3>;
    using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;

    Dynamics(int num_iters, double dt, double mu)
            : inverse_dynamics(inertia_properties, motion_transforms),
              jsim(inertia_properties, force_transforms),
              num_iters(num_iters),
              dt(dt),
              mu(mu) {}

    void print(std::ostream& os, const Params& params) const;

    void build_map() override;
    void evaluate(const Params& params) override;

    [[nodiscard]] auto get_f() const { return f; }
    [[nodiscard]] auto get_foot_pos() const { return foot_pos; }
    [[nodiscard]] auto get_df_dx() const { return df_dx; }
    [[nodiscard]] auto get_df_du() const { return df_du; }

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
    Action tau;
    Scalar active;

    State f;
    Eigen::Vector3d foot_pos;
    Eigen::Matrix<double, state_dims, state_dims, Eigen::RowMajor> df_dx;
    Eigen::Matrix<double, state_dims, action_dims, Eigen::RowMajor> df_du;

    std::tuple<JointState, JointState> step() const;
    [[nodiscard]] Vector3 prox(const Vector3& p) const;

    Vector3 compute_foot_pos() const;
};

#undef INPUT_DIMS
#undef OUTPUT_DIMS
#undef BASE


#endif //TO_IHC_2_DYNAMICS_H
