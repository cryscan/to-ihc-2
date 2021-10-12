//
// Created by cryscan on 9/19/21.
//

#include "stabilizer.h"

using namespace Robot;

Stabilizer::Stabilizer(const std::string& name, const ::State& gain) :
        Base(name, false),
        gain(gain.template cast<Scalar>()) {}

void Stabilizer::build_map() {
    CppAD::Independent(ad_x);

    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(q, ad_x, it, joint_space_dims)
    ASSIGN_VECTOR(u, ad_x, it, joint_space_dims)
    ASSIGN_VECTOR(q_star, ad_x, it, joint_space_dims)
    ASSIGN_VECTOR(d, ad_x, it, num_contacts)

    ad_y = pd();

    ad_fun.Dependent(ad_x, ad_y);
    ad_fun.optimize("no_compare_op");
}

void Stabilizer::evaluate(const Base::Params& params, EvalOption option) {
    Eigen::VectorXd x(input_dims);
    Eigen::VectorXd y(output_dims);

    x << params.x, params.q_star, params.d;
    y = model->ForwardZero(x);

    f = y;
}

Stabilizer::Action Stabilizer::pd() const {
    State dx;
    dx << q_star - q, -u;
    dx = dx.cwiseProduct(gain);

    JointState dq = dx.head<joint_space_dims>();
    JointState du = dx.tail<joint_space_dims>();

    return dq.tail<action_dims>() + du.tail<action_dims>();
}