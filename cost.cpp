//
// Created by cryscan on 8/18/21.
//

#include "cost.h"

Cost::Cost(const std::string& name) :
        Base(name),
        f(0) {}

Cost::Scalar Cost::cost() const {
    Scalar c = 0;
    c += 0.5 * (x_ - x_star).dot(scale_x.asDiagonal() * (x_ - x_star));
    c += 0.5 * u.dot(scale_u.asDiagonal() * u);
    return c;
}

void Cost::build_map() {
    CppAD::Independent(ad_x);

    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(x_, ad_x, it, state_dims)
    ASSIGN_VECTOR(u, ad_x, it, action_dims)
    ASSIGN_VECTOR(x_star, ad_x, it, state_dims)
    ASSIGN_VECTOR(scale_x, ad_x, it, state_dims)
    ASSIGN_VECTOR(scale_u, ad_x, it, action_dims)

    ad_y(0) = cost();
    ad_fun.Dependent(ad_x, ad_y);
    ad_fun.optimize("no_compare_op");

    ADVector ad_x_ = ad_x.cast<typename ScalarTraits::AD>();
    CppAD::Independent(ad_x_);

    build_jacobian();
}

void Cost::evaluate(const Params& params, EvalOption option) {
    using Jacobian = Eigen::RowVectorXd;

    Eigen::VectorXd x(input_dims + param_dims);
    Eigen::VectorXd y(output_dims);

    x << params.x, params.u, params.x_star, params.scale_x, params.scale_u;
    y = model->ForwardZero(x);
    f = y(0);

    if (option == EvalOption::FIRST_ORDER) {
        Jacobian jacobian = jacobian_model->ForwardZero(x);
        Eigen::DenseIndex it = 0;
        ASSIGN_COLS(df_dx, jacobian, it, state_dims)
        ASSIGN_COLS(df_du, jacobian, it, action_dims)

        df_dxx = params.scale_x.asDiagonal();
        df_duu = params.scale_u.asDiagonal();
    }
}
