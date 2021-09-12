//
// Created by cryscan on 8/18/21.
//

#include "cost.h"

Cost::Scalar Cost::cost() const {
    Scalar c = 0;
    c += 0.5 * (x - x_star).dot(scale_state.asDiagonal() * (x - x_star));
    c += 0.5 * u.dot(scale_action.asDiagonal() * u);
    return c;
}

void Cost::build_map() {
    CppAD::Independent(ad_x);

    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(x, ad_x, it, state_dims)
    ASSIGN_VECTOR(u, ad_x, it, action_dims)
    ASSIGN_VECTOR(x_star, ad_x, it, state_dims)

    ad_y(0) = cost();
    ad_fun.Dependent(ad_x, ad_y);
    ad_fun.optimize("no_compare_op");
}

void Cost::evaluate(const Params& params, EvalOption option) {
    using Jacobian = Eigen::RowVectorXd;

    Eigen::VectorXd x0(input_dims);
    Eigen::VectorXd y(output_dims);

    x0 << params.x, params.u, params.x_star;
    y = model->ForwardZero(x0);
    f = y(0);

    if (option == EvalOption::FIRST_ORDER) {
        Jacobian jac = model->Jacobian(x0);
        Eigen::DenseIndex it = 0;
        ASSIGN_COLS(df_dx, jac, it, state_dims)
        ASSIGN_COLS(df_du, jac, it, action_dims)
    }
}
