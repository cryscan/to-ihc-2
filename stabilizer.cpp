//
// Created by cryscan on 9/19/21.
//

#include "stabilizer.h"

Stabilizer::Stabilizer(const std::string& name) :
        ADBase(name),
        ContactBase(jacobians, inertia_properties),
        inverse_dynamics(inertia_properties, motion_transforms),
        jsim(inertia_properties, force_transforms) {}

void Stabilizer::build_map() {
    CppAD::Independent(ad_x);

    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(x, ad_x, it, state_dims)
    ASSIGN_VECTOR(x_star, ad_x, it, state_dims)
    ASSIGN_VECTOR(d, ad_x, it, num_contacts)

    ad_fun.Dependent(ad_x, ad_y);
    ad_fun.optimize("no_compare_op");
}

void Stabilizer::evaluate(const Base::Params& params, EvalOption option) {
    Eigen::VectorXd x0(input_dims);
    Eigen::VectorXd y(output_dims);

    x0 << params.x, params.x_star, params.d;
    y = model->ForwardZero(x0);

    f = y;
}
