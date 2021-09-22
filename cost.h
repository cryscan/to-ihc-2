//
// Created by cryscan on 8/18/21.
//

#ifndef TO_IHC_2_COST_H
#define TO_IHC_2_COST_H

#include "common.h"

struct Cost;

template<>
struct Parameter<Cost> {
    State x, x_star;
    Action u;
};

struct Cost : public ADBase<Cost, state_dims + state_dims + action_dims, 1> {
    using Base = decltype(base_type())::type;
    using Base::Params;

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::State;
    using Base::Action;

    using Base::input_dims;
    using Base::output_dims;

    Cost(const std::string& name, const ::State& scale_state, const ::Action& scale_action);

    void build_map() override;
    void evaluate(const Params& params, EvalOption option) override;

    [[nodiscard]] auto get_f() const { return f; }
    [[nodiscard]] auto get_df_dx() const { return df_dx; }
    [[nodiscard]] auto get_df_du() const { return df_du; }
    [[nodiscard]] auto get_df_dxx() const { return df_dxx; }
    [[nodiscard]] auto get_df_duu() const { return df_duu; }

    // const Eigen::Matrix<double, state_dims, 1> scale_state;
    // const Eigen::Matrix<double, action_dims, 1> scale_action;

private:
    const Robot::rcg::Matrix<state_dims, 1> scale_state;
    const Robot::rcg::Matrix<action_dims, 1> scale_action;

    State x, x_star;
    Action u;

    double f;
    Eigen::Matrix<double, 1, state_dims> df_dx;
    Eigen::Matrix<double, 1, action_dims> df_du;
    Eigen::Matrix<double, state_dims, state_dims> df_dxx;
    Eigen::Matrix<double, action_dims, action_dims> df_duu;

    [[nodiscard]] Scalar cost() const;
};

#endif //TO_IHC_2_COST_H
