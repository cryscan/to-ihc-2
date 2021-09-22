//
// Created by cryscan on 8/18/21.
//

#include "cost.h"

Cost::Cost(const std::string& name, const ::State& scale_state, const ::Action& scale_action) :
        Base(name),
        scale_state(scale_state.template cast<Scalar>()),
        scale_action(scale_action.template cast<Scalar>()),
        f(0),
        df_dxx(scale_state.asDiagonal()),
        df_duu(scale_action.asDiagonal()) {}

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
        Jacobian jacobian = model->Jacobian(x0);
        Eigen::DenseIndex it = 0;
        ASSIGN_COLS(df_dx, jacobian, it, state_dims)
        ASSIGN_COLS(df_du, jacobian, it, action_dims)
    }
}
