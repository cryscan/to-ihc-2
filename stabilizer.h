//
// Created by cryscan on 9/19/21.
//

#ifndef TO_IHC_2_STABILIZER_H
#define TO_IHC_2_STABILIZER_H

#include "dynamics.h"

struct Stabilizer;

template<>
struct Parameter<Stabilizer> {
    State x;
    JointState q_star;
    Contact d;
};

struct Stabilizer :
        public ADBase<Stabilizer, state_dims + joint_space_dims + num_contacts, action_dims>,
        public ContactBase {
    using Base = decltype(base_type())::type;

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::JointState;
    using Base::State;
    using Base::Action;

    template<typename StateVector, typename ActionVector>
    Stabilizer(const std::string& name,
               const Eigen::MatrixBase<StateVector>& pd_scale,
               const Eigen::MatrixBase<ActionVector>& id_scale) :
            Base(name),
            ContactBase(jacobians, inertia_properties),
            inverse_dynamics(inertia_properties, motion_transforms),
            jsim(inertia_properties, force_transforms),
            pd_scale(pd_scale.template cast<Scalar>()),
            id_scale(id_scale.template cast<Scalar>()) {}

    void build_map() override;
    void evaluate(const Params& params, EvalOption option) override;

    [[nodiscard]] auto get_f() const { return f; }

private:
    mutable Robot::rcg::HomogeneousTransforms transforms;
    mutable Robot::rcg::MotionTransforms motion_transforms;
    mutable Robot::rcg::ForceTransforms force_transforms;

    mutable Robot::rcg::Jacobians jacobians;
    mutable Robot::rcg::InertiaProperties inertia_properties;
    mutable Robot::rcg::InverseDynamics inverse_dynamics;
    mutable Robot::rcg::JSIM jsim;

    ::Action f;

    JointState q, u, q_star;
    Contact d;

    const State pd_scale;
    const Action id_scale;

    Action pd() const;
    Action id() const;
};

#endif //TO_IHC_2_STABILIZER_H
