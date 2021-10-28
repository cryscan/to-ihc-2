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
        public ADBase<Stabilizer, 0, state_dims + joint_space_dims + num_contacts, action_dims>,
        public ContactBase {
    using Base = decltype(base_type())::type;

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::JointState;
    using Base::State;
    using Base::Action;

    Stabilizer(const std::string& name, const ::State& gain);

    void build_map() override;
    void evaluate(const Params& params, EvalOption option) override;

    [[nodiscard]] auto get_f() const { return f; }

private:
    ::Action f;

    JointState q, u, q_star;
    Contact d;

    const State gain;

    Action pd() const;
};

#endif //TO_IHC_2_STABILIZER_H
