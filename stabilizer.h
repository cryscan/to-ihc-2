//
// Created by cryscan on 9/19/21.
//

#ifndef TO_IHC_2_STABILIZER_H
#define TO_IHC_2_STABILIZER_H

#include "dynamics.h"

struct Stabilizer;

template<>
struct Parameter<Stabilizer> {
    State x, x_star;
    Contact d;
};

struct Stabilizer :
        public ADBase<Stabilizer, state_dims + state_dims + num_contacts, action_dims>,
        public ContactBase {
    using Base = decltype(base_type())::type;

    using Base::State;
    using Base::Action;

    explicit Stabilizer(const std::string& name);

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

    State x, x_star;
    Contact d;
};

#endif //TO_IHC_2_STABILIZER_H
