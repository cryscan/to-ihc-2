#include <iostream>
#include <fstream>

#include "dynamics.h"
#include "cost.h"
#include "lqr.h"

using namespace Hopper;

Cost make_cost(std::istream& is, const State& x_star) {
    Eigen::VectorXd scale_state(8);
    for (int i = 0; i < scale_state.size(); ++i)
        is >> scale_state(i);

    Eigen::Vector2d scale_action;
    is >> scale_action(0) >> scale_action(1);

    Cost cost(scale_state, scale_action);
    cost.params.x_star = x_star;

    return cost;
}

int main() {
    std::fstream is("in.txt", std::ios::in);

    int horizon, interval, num_iters;
    is >> horizon >> interval >> num_iters;

    double defect_limit;
    is >> defect_limit;

    Dynamics dynamics(50, 0.01, 1, 60);
    for (int i = 0; i < dynamics.params.x.size(); ++i)
        is >> dynamics.params.x(i);
    is >> dynamics.params.u(0) >> dynamics.params.u(1);

    State x_star;
    for (int i = 0; i < x_star.size(); ++i)
        is >> x_star(i);

    Cost cost = make_cost(is, x_star);
    Cost cost_final = make_cost(is, x_star);

    LQR lqr(horizon, interval, {1.0, 0.5, 0.25, 0.125}, 4, dynamics, cost, cost_final);
    lqr.init_linear_interpolation();

    {
        std::cout << "initialization finished" << std::endl;

        std::fstream os("out.txt", std::ios::out | std::ios::trunc);
        lqr.print(os);
    }

    for (int i = 0; i < num_iters; ++i) {
        lqr.iterate();

        std::cout << "iter " << i << ":\t"
                  << lqr.total_cost() << '\t'
                  << lqr.total_defect() << '\t'
                  << lqr.get_decrease_ratio() << std::endl;

        if (defect_limit < 0.0 || lqr.total_defect() < defect_limit) {
            std::fstream os("out.txt", std::ios::out | std::ios::trunc);
            lqr.print(os);
        }
    }

    return 0;
}
