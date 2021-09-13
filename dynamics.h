//
// Created by cryscan on 8/17/21.
//

#ifndef TO_IHC_2_DYNAMICS_H
#define TO_IHC_2_DYNAMICS_H

#include "common.h"

struct Dynamics;

template<>
struct Parameter<Dynamics> {
    State x;
    Action u;
    double d{0};
};

#define INPUT_DIMS  (state_dims + action_dims + 1)
#define OUTPUT_DIMS state_dims

struct Dynamics : public ADBase<Dynamics, INPUT_DIMS, OUTPUT_DIMS> {
    using Base = decltype(base_type())::type;

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::JointState;
    using Base::Action;

    using Base::input_dims;
    using Base::output_dims;

    using Vector3 = Hopper::rcg::Vector3;
    using Matrix3 = Hopper::rcg::Matrix<3, 3>;

    Dynamics(const std::string& name, int num_iters, double dt, double mu, double torque_limit);

    void build_map() override;
    void evaluate(const Params& params, EvalOption option) override;

    [[nodiscard]] auto get_f() const { return f; }
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
    Scalar d;

    State f;
    Eigen::Matrix<double, state_dims, state_dims, Eigen::RowMajor> df_dx;
    Eigen::Matrix<double, state_dims, action_dims, Eigen::RowMajor> df_du;

    inline std::tuple<JointState, JointState> step() const;
    inline Vector3 prox(const Vector3& p) const;
};

#undef INPUT_DIMS
#undef OUTPUT_DIMS


#endif //TO_IHC_2_DYNAMICS_H
