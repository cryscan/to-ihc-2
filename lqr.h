//
// Created by cryscan on 8/18/21.
//

#ifndef TO_IHC_2_LQR_H
#define TO_IHC_2_LQR_H

#include <iostream>

#include "common.h"

class Dynamics;

class Cost;

struct LQR {
    static constexpr int ext_state_dims = state_dims + 1;
    static constexpr int ext_action_dims = action_dims + 1;

    using ExtendedState = Eigen::Matrix<double, ext_state_dims, 1>;
    using ExtendedAction = Eigen::Matrix<double, ext_action_dims, 1>;

    using A = Eigen::Matrix<double, ext_state_dims, ext_state_dims, Eigen::RowMajor>;
    using B = Eigen::Matrix<double, ext_state_dims, action_dims, Eigen::RowMajor>;
    using Q = Eigen::Matrix<double, ext_state_dims, ext_state_dims>;
    using R = Eigen::Matrix<double, action_dims, action_dims>;
    using P = Eigen::Matrix<double, ext_state_dims, ext_state_dims>;
    using K = Eigen::Matrix<double, action_dims, ext_state_dims>;

    LQR(int horizon, int interval, std::vector<double> steps, const Dynamics& dynamics, const Cost& cost);

    void init(const std::vector<State>& x, const std::vector<Action>& u);
    void init_linear_interpolation();

    void iterate();

    // computes the overall cost starting from the time index
    // assuming that the rollout has been updated
    [[nodiscard]] double total_cost() const;
    [[nodiscard]] double total_defect() const;

    void print(std::ostream& os) const;

    const int horizon;
    const int interval;
    const std::vector<double> steps;

private:
    std::vector<State> x;
    std::vector<Action> u;

    std::vector<A> a;
    std::vector<B> b;
    std::vector<Q> q;
    std::vector<R> r;
    std::vector<P> p;
    std::vector<K> k;

    std::vector<Eigen::Vector2d> dv;
    double mu;
    double delta;

    std::vector<Dynamics> vec_dynamics;
    std::vector<Cost> vec_cost;

    void linearize();
    void solve();
    void update();
};

#endif //TO_IHC_2_LQR_H
