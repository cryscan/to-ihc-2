//
// Created by cryscan on 9/11/21.
//

#ifndef TO_IHC_2_KINETICS_H
#define TO_IHC_2_KINETICS_H

#include "common.h"

struct Kinetics;

template<>
struct Parameter<Kinetics> {
    State x;
};

#define INPUT_DIMS  (state_dims)
#define OUTPUT_DIMS 9

struct Kinetics : public ADBase<Kinetics, INPUT_DIMS, OUTPUT_DIMS> {
    using Base = decltype(base_type());

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::JointState;
    using Base::Action;

    using Base::input_dims;
    using Base::output_dims;

    using Vector3 = Hopper::rcg::Vector3;
    using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;

    void build_map() override;
    void evaluate(const Params& params, EvalOption option) override;

    Eigen::Vector3d get_body_pos() const { return body_pos; }
    Eigen::Vector3d get_knee_pos() const { return knee_pos; }
    Eigen::Vector3d get_foot_pos() const { return foot_pos; }

private:
    mutable Hopper::rcg::HomogeneousTransforms transforms;

    JointState q, u;

    Eigen::Vector3d body_pos;
    Eigen::Vector3d knee_pos;
    Eigen::Vector3d foot_pos;

    inline Vector3 compute_body_pos() const;
    inline Vector3 compute_knee_pos() const;
    inline Vector3 compute_foot_pos() const;
};

#undef INPUT_DIMS
#undef OUTPUT_DIMS

#endif //TO_IHC_2_KINETICS_H
