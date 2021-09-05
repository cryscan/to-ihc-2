//
// Created by cryscan on 8/17/21.
//

#ifndef TO_IHC_2_DYNAMICS_H
#define TO_IHC_2_DYNAMICS_H

#include <iostream>
#include <Eigen/Geometry>

#include "common.h"

struct DynamicsParams {
    State x;
    Action u;
    double active{0};
};

#define INPUT_DIMS  (state_dims + action_dims + 1)
#define OUTPUT_DIMS state_dims

struct Dynamics : public ADBase<DynamicsParams, INPUT_DIMS, OUTPUT_DIMS> {
    using Base = decltype(base_type());

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::JointState;
    using Base::Action;

    using Base::input_dims;
    using Base::output_dims;

    using Vector3 = Hopper::rcg::Vector3;
    using Matrix3 = Hopper::rcg::Matrix<3, 3>;
    using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;

    Dynamics(int num_iters, double dt, double mu, double torque_limit) :
            inverse_dynamics(inertia_properties, motion_transforms),
            jsim(inertia_properties, force_transforms),
            num_iters(num_iters),
            dt(dt),
            mu(mu),
            torque_limit(torque_limit),
            ad_torque_limit(torque_limit) {}

    void build_map() override;
    void evaluate(const Params& params, EvalOption option) override;
    void evaluate_extra();

    [[nodiscard]] auto get_f() const { return f; }
    [[nodiscard]] auto get_foot_pos() const { return foot_pos; }
    [[nodiscard]] auto get_df_dx() const { return df_dx; }
    [[nodiscard]] auto get_df_du() const { return df_du; }

    const double torque_limit;

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
    const Scalar ad_torque_limit;

    JointState q, u;
    Action tau;
    Scalar active;

    Hopper::rcg::Matrix<Eigen::Dynamic, 1> ad_y_extra;
    CppAD::ADFun<double> ad_fun_extra;

    State f;
    Eigen::Vector3d foot_pos;
    Eigen::Matrix<double, state_dims, state_dims, Eigen::RowMajor> df_dx;
    Eigen::Matrix<double, state_dims, action_dims, Eigen::RowMajor> df_du;

    inline std::tuple<JointState, JointState> step() const;
    inline Vector3 prox(const Vector3& p) const;

    inline Vector3 compute_foot_pos() const;

    inline void prepare_map();
};

#undef INPUT_DIMS
#undef OUTPUT_DIMS


#endif //TO_IHC_2_DYNAMICS_H
