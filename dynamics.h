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
    Contact d;
};

#define INPUT_DIMS  (state_dims + action_dims + num_contacts)
#define OUTPUT_DIMS state_dims

struct Dynamics : public ADBase<Dynamics, INPUT_DIMS, OUTPUT_DIMS> {
    using Base = decltype(base_type())::type;

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::State;
    using Base::Action;

    using Base::input_dims;
    using Base::output_dims;

    using Vector3 = Robot::rcg::Vector3;
    using Matrix3 = Robot::rcg::Matrix<3, 3>;

    using ContactJacobian = Robot::rcg::Matrix<contact_dims, joint_space_dims>;
    using ContactJacobianTranspose = Robot::rcg::Matrix<joint_space_dims, contact_dims>;
    using ContactInertia = Robot::rcg::Matrix<contact_dims, contact_dims>;
    using Percussion = Robot::rcg::Matrix<contact_dims, 1>;

    Dynamics(const std::string& name, int num_iters, double dt, double mu, double torque_limit);

    void build_map() override;
    void evaluate(const Params& params, EvalOption option) override;

    [[nodiscard]] auto get_f() const { return f; }
    [[nodiscard]] auto get_df_dx() const { return df_dx; }
    [[nodiscard]] auto get_df_du() const { return df_du; }

protected:
    mutable Robot::rcg::HomogeneousTransforms transforms;
    mutable Robot::rcg::MotionTransforms motion_transforms;
    mutable Robot::rcg::ForceTransforms force_transforms;

    mutable Robot::rcg::Jacobians jacobians;
    mutable Robot::rcg::InertiaProperties inertia_properties;
    mutable Robot::rcg::InverseDynamics inverse_dynamics;
    mutable Robot::rcg::JSIM jsim;

    const int num_iters;
    const Scalar dt;
    const Scalar mu;
    const Scalar torque_limit;

    JointState q, u;
    Action tau;
    Contact d;

    ::State f;
    Eigen::Matrix<double, state_dims, state_dims, Eigen::RowMajor> df_dx;
    Eigen::Matrix<double, state_dims, action_dims, Eigen::RowMajor> df_du;

    std::tuple<JointState, JointState> step() const;
    std::tuple<JointState, JointState> contact(const JointState& qm, const JointState& h) const;
    Vector3 prox(const Vector3& p) const;
};

#undef INPUT_DIMS
#undef OUTPUT_DIMS


#endif //TO_IHC_2_DYNAMICS_H
