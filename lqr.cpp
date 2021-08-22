//
// Created by cryscan on 8/18/21.
//

#include <cassert>

#include "lqr.h"
#include "dynamics.h"
#include "cost.h"

LQR::LQR(int horizon, double beta, int max_trial, Dynamics& dynamics, Cost& cost)
        : horizon(horizon),
          beta(beta),
          max_trial(max_trial),
          dynamics(dynamics),
          cost(cost),
          x(horizon + 1, dynamics.params.x),
          u(horizon, dynamics.params.u),
          a(horizon, A::Identity()),
          b(horizon, B::Zero()),
          q(horizon, Q::Zero()),
          r(horizon, R::Zero()),
          p(horizon + 1, P::Zero()),
          k(horizon, K::Zero()) {
    assert(horizon > 0);

    dynamics.build_map();
    cost.build_map();
}

void LQR::action_rollout(std::vector<State>& x_, const std::vector<Action>& u_, int begin, int end) {
    assert(begin >= 0);
    assert(end <= horizon);

    for (int i = begin; i < end; ++i) {
        dynamics.params.x = x_[i];
        dynamics.params.u = u_[i];

        dynamics.evaluate_extra();
        dynamics.params.active = dynamics.get_foot_pos()(2) <= 0;

        dynamics.Base::evaluate();
        x_[i + 1] = dynamics.get_f();

        // linearize the system around the new trajectory
        a[i].topLeftCorner<state_dims, state_dims>() = dynamics.get_df_dx();
        b[i].topRows<state_dims>() = dynamics.get_df_du();

        // quadratically approximate the cost
        cost.params.x = x_[i];
        cost.params.u = u_[i];
        cost.params.x_prev = x[i];
        cost.params.u_prev = u[i];
        cost.Base::evaluate();

        q[i] << cost.get_df_dxx(), cost.get_df_dx().transpose(),
                cost.get_df_dx(), cost.get_f();
        r[i] = cost.get_df_duu();
    }
}

void LQR::nominal_rollout() {
    action_rollout(x, u, 0, horizon);
}

void LQR::rollout() {
    std::vector<State> x_(x);
    std::vector<Action> u_(u);

    for (int i = 0; i < horizon; ++i) {
        auto z = (ExtendedState() << x_[i] - x[i], 1.0).finished();
        Action v = k[i] * z;
        u_[i] = u[i] + v;

        action_rollout(x_, u_, i, i + 1);
    }

    std::swap(x, x_);
    std::swap(u, u_);
}

void LQR::solve() {
    for (int i = horizon - 1; i >= 0; --i) {
        K bt_p = b[i].transpose() * p[i + 1];
        k[i] = -(r[i] + bt_p * b[i]).ldlt().solve(bt_p * a[i]);

        A a_bk = a[i] + b[i] * k[i];
        P kt_r_k = k[i].transpose() * r[i] * k[i];
        p[i] = q[i] + kt_r_k + a_bk.transpose() * p[i + 1] * a_bk;
    }
}

double LQR::cost_sum(int index) const {
    assert(index >= 0);
    assert(index < horizon);

    double c = 0;
    for (int i = index; i < horizon; ++i) {
        c += q[i].bottomRightCorner<1, 1>()(0);
    }
    return c;
}

Action LQR::cost_sum_derivative(int index, const std::vector<Action>& u_) const {
    assert(index >= 0);
    assert(index < horizon);

    Action d = r[index] * u_[index];
    Eigen::Matrix<double, state_dims, action_dims> head = b[index].topRows<state_dims>();

    // accumulates the derivatives throughout the time-steps
    for (int i = index; i < horizon; ++i) {
        if (i > index) head = a[i].topLeftCorner<state_dims, state_dims>() * head;
        d += head.transpose() * q[i].topRightCorner<state_dims, 1>();
    }

    return d;
}

void LQR::print(std::ostream& os) const {
    for (int i = 0; i < horizon; ++i)
        os << x[i].transpose() << '\t' << u[i].transpose() << std::endl;
}
