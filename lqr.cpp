//
// Created by cryscan on 8/18/21.
//

#include <cassert>
#include <limits>

#include "lqr.h"
#include "dynamics.h"
#include "cost.h"

LQR::LQR(int horizon, std::vector<double> steps, Dynamics& dynamics, Cost& cost)
        : horizon(horizon),
          steps(std::move(steps)),
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
    dynamics.build_map();
    cost.build_map();
}

void LQR::init(const std::vector<State>& x_, const std::vector<Action>& u_) {
    std::copy(x_.begin(), x_.end(), x.begin());
    std::copy(u_.begin(), u_.end(), u.begin());
}

void LQR::init_linear_interpolation() {
    State dx = cost.params.x_star - x[0];
    for (int i = 0; i < horizon; ++i)
        x[i + 1] = x[i] + dx / horizon;
}

void LQR::linearize() {
    for (int i = 0; i < horizon; ++i) {
        dynamics.params.x = x[i];
        dynamics.params.u = u[i];

        dynamics.evaluate_extra();
        dynamics.params.active = dynamics.get_foot_pos()(2) <= 0;

        dynamics.Base::evaluate();

        // linearize the system around the new trajectory
        a[i].topLeftCorner<state_dims, state_dims>() = dynamics.get_df_dx();
        a[i].topRightCorner<state_dims, 1>() = dynamics.get_f() - x[i + 1];
        b[i].topRows<state_dims>() = dynamics.get_df_du();

        // quadratically approximate the cost
        cost.params.x = x[i];
        cost.params.u = u[i];
        cost.Base::evaluate();

        q[i] << cost.get_df_dxx(), cost.get_df_dx().transpose(),
                cost.get_df_dx(), cost.get_f();
        r[i] = cost.get_df_duu();
    }
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

/*
void LQR::update() {
    std::vector<State> x_(x);
    std::vector<Action> u_(u);

    for (int i = 0; i < horizon; ++i) {
        ExtendedState z;
        z << x_[i] - x[i], 1.0;

        u_[i] += k[i] * z;

        ExtendedState dx = (a[i] + b[i] * k[i]) * z;
        x_[i + 1] += dx.head<state_dims>();
    }

    x.swap(x_);
    u.swap(u_);
}
 */

void LQR::update() {
    std::vector<State> x_(x);
    std::vector<Action> u_(u);

    std::vector<State> dx(horizon + 1, State::Zero());
    std::vector<Action> du(horizon, Action::Zero());

    double min_cost = std::numeric_limits<double>::max();

    for (int i = 0; i < horizon; ++i) {
        ExtendedState z;
        z << dx[i], 1.0;

        du[i] = k[i] * z;

        ExtendedState dz = (a[i] + b[i] * k[i]) * z;
        dx[i + 1] = dz.head<state_dims>();
    }

#pragma omp parallel default(none) shared(min_cost, steps, x_, u_, dx, du)
    {
        // std::vector<State> local_x(horizon);
        // std::vector<Action> local_u(horizon);
        CostParams params = cost.params;

        double local_cost = 0;

#pragma omp for
        for (double alpha: steps) {
            for (int i = 0; i < horizon; ++i) {
                params.x = x[i] + alpha * dx[i];
                params.u = u[i] + alpha * du[i];
                local_cost += generic_cost<double>(params.x,
                                                   params.x_star,
                                                   params.u,
                                                   cost.scale_state,
                                                   cost.scale_action);
            }

#pragma omp critical
            {
                if (local_cost < min_cost) {
                    min_cost = local_cost;
                    for (int i = 0; i < horizon; ++i) {
                        x_[i] = x[i] + alpha * dx[i];
                        u_[i] = u[i] + alpha * du[i];
                    }
                }
            }
        }

    }

    x.swap(x_);
    u.swap(u_);
}


void LQR::iterate() {
    linearize();
    solve();
    update();
}

double LQR::total_cost() const {
    double c = 0;
    for (int i = 0; i < horizon; ++i)
        c += q[i].bottomRightCorner<1, 1>()(0);
    return c;
}

double LQR::total_defect() const {
    double d = 0;
    for (int i = 0; i < horizon; ++i)
        d += a[i].topRightCorner<state_dims, 1>().squaredNorm();
    return d;
}

void LQR::print(std::ostream& os) const {
    for (int i = 0; i < horizon; ++i)
        os << x[i].transpose() << '\t' << u[i].transpose() << std::endl;
}