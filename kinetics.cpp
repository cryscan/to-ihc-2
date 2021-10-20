//
// Created by cryscan on 9/11/21.
//

#include "kinetics.h"

Kinetics::Kinetics(const std::string& name) : Base(name) {}

void Kinetics::build_map() {
    CppAD::Independent(ad_x);

    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(q, ad_x, it, joint_space_dims)
    ASSIGN_VECTOR(u, ad_x, it, joint_space_dims)

    it = 0;
    FILL_VECTOR(ad_y, compute_body_pos(), it, 3)
    FILL_VECTOR(ad_y, compute_knee_pos(), it, 3)
    FILL_VECTOR(ad_y, compute_foot_pos(), it, 3)

    ad_fun.Dependent(ad_x, ad_y);
    ad_fun.optimize("no_compare_op");
}

void Kinetics::evaluate(const ADBase::Params& params, EvalOption option) {
    Eigen::VectorXd x(input_dims + param_dims);
    Eigen::VectorXd y(output_dims);

    x << params.x;

    y = models[0]->ForwardZero(x);
    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(body_pos, y, it, 3)
    ASSIGN_VECTOR(knee_pos, y, it, 3)
    ASSIGN_VECTOR(foot_pos, y, it, 3)
}

Kinetics::Vector3 Kinetics::compute_body_pos() const {
    Affine3 body(transforms.fr_u0_X_body(q));
    return body.translation();
}

Kinetics::Vector3 Kinetics::compute_knee_pos() const {
    Affine3 knee(transforms.fr_u0_X_knee(q));
    return knee.translation();
}

Kinetics::Vector3 Kinetics::compute_foot_pos() const {
    Affine3 foot(transforms.fr_u0_X_foot(q));
    return foot.translation();
}
