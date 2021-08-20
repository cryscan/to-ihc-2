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

    using A = Eigen::Matrix<double, ext_state_dims, ext_state_dims, Eigen::RowMajor>;
    using B = Eigen::Matrix<double, ext_state_dims, action_dims, Eigen::RowMajor>;
    using Q = Eigen::Matrix<double, ext_state_dims, ext_state_dims>;
    using R = Eigen::Matrix<double, action_dims, action_dims>;
    using P = Eigen::Matrix<double, ext_state_dims, ext_state_dims>;
    using K = Eigen::Matrix<double, action_dims, ext_state_dims>;

    using ExtendedState = Eigen::Matrix<double, ext_state_dims, 1>;

    LQR(int horizon, double beta, int max_trial, Dynamics& dynamics, Cost& cost);

    // generates a trajectory using the u provided and stores it into x
    // it also calculates derivatives
    void action_rollout(std::vector<State>& x_, const std::vector<Action>& u_, int begin, int end);
    void nominal_rollout();

    // generates x and u using LQR feedback control and line search
    void rollout();
    void solve();

    // computes the overall cost starting from the time index
    // assuming that the rollout has been updated
    [[nodiscard]] double cost_sum(int index) const;

    // computes the derivative of the overall cost w.r.t. the current action
    // at the time index and the action sequence u_
    // assuming that the rollout has been updated
    [[nodiscard]] Action cost_sum_derivative(int index, const std::vector<Action>& u_) const;

    void print(std::ostream& os) const;

    const int horizon;
    const double beta;
    const int max_trial;

private:
    std::vector<State> x;
    std::vector<Action> u;

    std::vector<A> a;
    std::vector<B> b;
    std::vector<Q> q;
    std::vector<R> r;
    std::vector<P> p;
    std::vector<K> k;

    Dynamics& dynamics;
    Cost& cost;
};

#endif //TO_IHC_2_LQR_H
