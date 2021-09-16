//
// Created by cryscan on 8/18/21.
//

#include <iostream>
#include <limits>
#include <omp.h>

#include "lqr.h"
#include "kinetics.h"
#include "dynamics.h"
#include "cost.h"

namespace {
    using CppAD::thread_alloc;

    bool in_parallel() { return omp_in_parallel() != 0; }

    size_t thread_num() { return static_cast<size_t>(omp_get_thread_num()); }

    const size_t num_threads = omp_get_max_threads();
}

LQR::LQR(int horizon, int interval, std::vector<double> line_search_steps, int max_line_search_trails,
         const Kinetics& kinetics, const Dynamics& dynamics, const Cost& cost, const Cost& cost_final) :
        horizon(horizon),
        interval(interval),
        line_search_steps(std::move(line_search_steps)),
        max_line_search_trails(max_line_search_trails),
        vec_kinetics(num_threads, kinetics),
        vec_dynamics(num_threads, dynamics),
        vec_cost(num_threads, cost),
        vec_cost_final(num_threads, cost_final),
        x(horizon + 1, dynamics.params.x),
        u(horizon, dynamics.params.u),
        a(horizon, A::Identity()),
        b(horizon, B::Zero()),
        q(horizon, Q::Zero()),
        r(horizon, R::Zero()),
        p(horizon + 1, P::Zero()),
        k(horizon, K::Zero()),
        dv(horizon),
        mu(0),
        delta(0),
        decrease_ratio(0) {
    thread_alloc::parallel_setup(omp_get_max_threads(), in_parallel, thread_num);
    thread_alloc::hold_memory(true);
    CppAD::parallel_ad<double>();

    for (auto& fun: vec_kinetics) fun.Base::build_map();
    for (auto& fun: vec_dynamics) fun.Base::build_map();
    for (auto& fun: vec_cost) fun.Base::build_map();
    for (auto& fun: vec_cost_final) fun.Base::build_map();
}

void LQR::init(const std::vector<State>& x_, const std::vector<Action>& u_) {
    std::copy(x_.begin(), x_.end(), x.begin());
    std::copy(u_.begin(), u_.end(), u.begin());
}

void LQR::init_linear_interpolation() {
    State dx = vec_cost[0].params.x_star - x[0];
    for (int i = 0; i < horizon; ++i)
        x[i + 1] = x[i] + dx / horizon;
}

void LQR::linearize() {
#pragma omp parallel for default(none)
    for (int i = 0; i < horizon; ++i) {
        size_t id = thread_num();
        auto& kinetics = vec_kinetics[id];
        auto& dynamics = vec_dynamics[id];
        auto& cost = vec_cost[id];

        kinetics.params.x = x[i];
        kinetics.Base::evaluate();

        dynamics.params.x = x[i];
        dynamics.params.u = u[i];
        dynamics.params.d = kinetics.get_foot_pos().z();
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
    {
        auto& cost = vec_cost_final[0];
        cost.params.x = x[horizon];
        cost.params.u = Action::Zero();
        cost.Base::evaluate();
        p[horizon] << cost.get_df_dxx(), cost.get_df_dx().transpose(), cost.get_df_dx(), cost.get_f();
    }
}

void LQR::solve() {
    static const double delta_zero = 2;
    static const double mu_min = 1e-6;

    bool ok = true;
    feedforward_gain = 0;

    do {
        for (int i = horizon - 1; i >= 0; --i) {
            K bt_p = b[i].transpose() * (p[i + 1] + mu * P::Identity());
            R q_uu = r[i] + bt_p * b[i];
            K q_ux_u = bt_p * a[i];

            if (q_uu.llt().info() == Eigen::NumericalIssue) {
                // numeric issue happens, break the loop.
                ok = false;
                break;
            }

            k[i] = -q_uu.ldlt().solve(q_ux_u);

            Action d = k[i].rightCols<1>();
            Action q_u = q_ux_u.rightCols<1>();
            dv[i] << d.dot(q_u), 0.5 * d.dot(q_uu * d);

            feedforward_gain += d.maxCoeff() / (u[i].norm() + 1);
            feedforward_gain /= horizon;

            A a_bk = a[i] + b[i] * k[i];
            P kt_r_k = k[i].transpose() * r[i] * k[i];
            p[i] = q[i] + kt_r_k + a_bk.transpose() * p[i + 1] * a_bk;
        }

        if (ok) {
            // decrease mu
            delta = std::min(1.0 / delta_zero, delta / delta_zero);
            mu = mu * delta < mu_min ? 0 : mu * delta;
        } else {
            // increase mu
            delta = std::max(delta_zero, delta * delta_zero);
            mu = std::max(mu_min, mu * delta);
        }
    } while (!ok);
}

void LQR::update() {
    std::vector<State> x_hat(x);
    std::vector<Action> u_hat(u);

    std::vector<State> x_(x);
    std::vector<Action> u_(u);

    std::vector<State> dx(x.size(), State::Zero());
    std::vector<Action> du(u.size(), Action::Zero());

    double last_cost = total_cost();
    double min_cost = std::numeric_limits<double>::max();

    bool ok = false;
    int trails = 0;
    auto steps = line_search_steps;

    do {
#pragma omp parallel for default(none) firstprivate(x_, u_, dx, du) shared(last_cost, min_cost, steps, x_hat, u_hat, ok)
        for (double alpha: steps) {
            size_t id = thread_num();
            auto& kinetics = vec_kinetics[id];
            auto& dynamics = vec_dynamics[id];

            double expected = 0;

            for (int i = 0; i < horizon; ++i) {
                expected += alpha * (dv[i](0) + alpha * dv[i](1));

                // override by simulation
                dx[i] = x_[i] - x[i];

                ExtendedState z;
                z << dx[i], 1.0;

                K l = feedback(i, alpha);
                du[i] = l * z;
                u_[i] = u[i] + du[i];

                kinetics.params.x = x_[i];
                kinetics.Base::evaluate();

                dynamics.params.x = x_[i];
                dynamics.params.u = u_[i];
                dynamics.params.d = kinetics.get_foot_pos().z();

                dynamics.Base::evaluate(EvalOption::ZERO_ORDER);
                x_[i + 1] = dynamics.get_f();
            }

            double local_cost = total_cost(x_, u_);

#pragma omp critical
            {
                if (local_cost < min_cost) {
                    min_cost = local_cost;
                    x_hat.swap(x_);
                    u_hat.swap(u_);

                    decrease_ratio = (local_cost - last_cost) / expected;
                    if (decrease_ratio > 0.0001)
                        ok = true;
                }
            }
        }

        if (!ok) {
            std::transform(steps.begin(), steps.end(), steps.begin(),
                           [](auto alpha) { return alpha / 16; });
            trails++;
        }
    } while (!(ok || trails >= max_line_search_trails));

    x.swap(x_hat);
    u.swap(u_hat);
}

void LQR::iterate() {
    linearize();
    solve();
    update();
}

LQR::K LQR::feedback(int i, double alpha) const {
    K l = k[i];
    l.rightCols<1>() *= alpha;
    return l;
}

double LQR::total_cost(const std::vector<State>& x_, const std::vector<Action>& u_) const {
    size_t id = thread_num();
    double c = 0;

    for (int i = 0; i < horizon; ++i) {
        auto& cost = vec_cost[id];
        cost.params.x = x_[i];
        cost.params.u = u_[i];
        cost.Base::evaluate(EvalOption::ZERO_ORDER);
        c += cost.get_f();
    }
    {
        auto& cost = vec_cost_final[id];
        cost.params.x = x_[horizon];
        cost.params.u = Action::Zero();
        cost.Base::evaluate(EvalOption::ZERO_ORDER);
        c += cost.get_f();
    }

    return c;
}

double LQR::total_cost() const {
    return total_cost(x, u);
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