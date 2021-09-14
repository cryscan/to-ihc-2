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

#define INPUT_DIMS  (state_dims + state_dims + action_dims)
#define OUTPUT_DIMS 1

struct Cost : public ADBase<Cost, INPUT_DIMS, OUTPUT_DIMS> {
    using Base = decltype(base_type())::type;
    using Base::Params;

    using Base::Scalar;
    using Base::ScalarTraits;
    using Base::JointState;
    using Base::Action;

    using Base::input_dims;
    using Base::output_dims;

    using State = Robot::rcg::Matrix<state_dims, 1>;

    template<typename StateVector, typename ActionVector>
    Cost(const std::string& name,
         const Eigen::MatrixBase<StateVector>& scale_state,
         const Eigen::MatrixBase<ActionVector>& scale_action) :
            Base(name),
            scale_state(scale_state.template cast<Scalar>()),
            scale_action(scale_action.template cast<Scalar>()),
            f(0),
            df_dxx(scale_state.asDiagonal()),
            df_duu(scale_action.asDiagonal()) {};

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

#undef INPUT_DIMS
#undef OUTPUT_DIMS

#endif //TO_IHC_2_COST_H
