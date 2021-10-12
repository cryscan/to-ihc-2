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

struct Kinetics : public ADBase<Kinetics, state_dims, 9> {
    using Base = decltype(base_type())::type;

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::JointState;
    using Base::Action;

    using Base::input_dims;
    using Base::output_dims;

    using Vector3 = Robot::rcg::Vector3;
    using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;

    explicit Kinetics(const std::string& name);

    void build_map() override;
    void evaluate(const Params& params, EvalOption option) override;

    Eigen::Vector3d get_body_pos() const { return body_pos; }
    Eigen::Vector3d get_knee_pos() const { return knee_pos; }
    Eigen::Vector3d get_foot_pos() const { return foot_pos; }

private:
    mutable Robot::rcg::HomogeneousTransforms transforms;

    JointState q, u;

    Eigen::Vector3d body_pos;
    Eigen::Vector3d knee_pos;
    Eigen::Vector3d foot_pos;

    inline Vector3 compute_body_pos() const;
    inline Vector3 compute_knee_pos() const;
    inline Vector3 compute_foot_pos() const;
};

#endif //TO_IHC_2_KINETICS_H
