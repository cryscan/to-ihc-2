#include <iostream>
#include <fstream>
#include <sstream>

#include "kinetics.h"
#include "dynamics.h"
#include "stabilizer.h"
#include "cost.h"
#include "lqr.h"

auto read_input_file() {
    std::fstream fs("in.txt", std::ios::in);
    std::string str, line;

    while (std::getline(fs, line)) {
        if (!line.empty() && line[0] != '#') {
            str.append(line + '\t');
        }
    }

    return std::stringstream(str);
}

auto make_dynamics(std::istream& is) {
    Dynamics dynamics("dynamics", 50);
    for (int i = 0; i < dynamics.params.x.size(); ++i)
        is >> dynamics.params.x(i);
    return dynamics;
}

auto make_cost(std::istream& is, const std::string& name, const State& x_star) {
    Eigen::VectorXd scale_state(8);
    for (int i = 0; i < scale_state.size(); ++i)
        is >> scale_state(i);

    Eigen::Vector2d scale_action;
    is >> scale_action(0) >> scale_action(1);

    Cost cost(name, scale_state, scale_action);
    cost.params.x_star = x_star;

    return cost;
}

std::tuple<std::vector<State>, std::vector<Action>>
read_init_trajectory(std::ifstream& fs) {
    std::vector<State> x;
    std::vector<Action> u;

    std::string line;
    while (std::getline(fs, line)) {
        std::stringstream ss(line);
        State x_;
        Action u_;

        for (int i = 0; i < x_.size(); ++i)
            ss >> x_(i);
        for (int i = 0; i < u_.size(); ++i) {
            ss >> u_(i);
        }

        x.push_back(x_);
        u.push_back(u_);
    }

    return {x, u};
}

void stabilizer_init_trajectory(std::istream& is, LQR& lqr, Kinetics& kinetics, Dynamics& dynamics) {
    State gain;
    for (int i = 0; i < gain.size(); ++i)
        is >> gain(i);

    Stabilizer stabilizer("stabilizer", gain);
    stabilizer.Base::build_map();

    State x0 = dynamics.params.x;
    stabilizer.params.q_star = x0.head<joint_space_dims>();

    int horizon = lqr.horizon;

    std::vector<State> x(horizon + 1, x0);
    std::vector<Action> u(horizon, Action::Zero());

    for (int i = 0; i < horizon; ++i) {
        stabilizer.params.x = x[i];
        stabilizer.Base::evaluate();
        u[i] = stabilizer.get_f();

        kinetics.params.x = x[i];
        kinetics.Base::evaluate();

        dynamics.params.x = x[i];
        dynamics.params.u = u[i];
        dynamics.params.d << kinetics.get_body_pos().z(), kinetics.get_knee_pos().z(), kinetics.get_foot_pos().z();
        dynamics.Base::evaluate();

        x[i + 1] = dynamics.get_f();
    }

    lqr.init(x, u);
}

auto make_lqr(std::istream& is,
              const Kinetics& kinetics,
              const Dynamics& dynamics,
              const Cost& cost,
              const Cost& cost_final) {
    int horizon, interval;
    is >> horizon;

    interval = horizon;

    LQR lqr(horizon, interval, {1.0, 0.5, 0.25, 0.125}, 3, 5, kinetics, dynamics, cost, cost_final);

//    std::ifstream fs(init_file);
//    auto[x, u] = read_init_trajectory(fs);
//    lqr.init(x, u);

    return lqr;
}

int main() {
    auto is = read_input_file();

    int num_iters;
    is >> num_iters;

    double feedforward_gain_limit;
    is >> feedforward_gain_limit;

    Kinetics kinetics("kinetics");
    auto dynamics = make_dynamics(is);

    kinetics.Base::build_map();
    dynamics.Base::build_map();

    State x_star;
    for (int i = 0; i < x_star.size(); ++i)
        is >> x_star(i);

    auto cost = make_cost(is, "cost", x_star);
    auto cost_final = make_cost(is, "cost_final", x_star);

    auto lqr = make_lqr(is, kinetics, dynamics, cost, cost_final);

    stabilizer_init_trajectory(is, lqr, kinetics, dynamics);

    {
        std::cout << "Initialization finished" << std::endl;

        std::fstream os("out.txt", std::ios::out | std::ios::trunc);
        lqr.print(os);
    }

    for (int i = 0; i < num_iters; ++i) {
        lqr.iterate();

        std::cout << "iter " << i << ":\t"
                  << lqr.total_cost() << '\t'
                  << lqr.get_decrease_ratio() << '\t'
                  << lqr.get_feedforward_gain() << std::endl;

        {
            std::fstream os("out.txt", std::ios::out | std::ios::trunc);
            lqr.print(os);
        }

        if (lqr.get_feedforward_gain() < feedforward_gain_limit) {
            std::cout << "Converged" << std::endl;
            break;
        }
    }

    return 0;
}
