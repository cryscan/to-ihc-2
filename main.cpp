#include <iostream>
#include <fstream>

#include "dynamics.h"
#include "cost.h"
#include "lqr.h"

using namespace Hopper;

int main() {
    std::fstream is("in.txt", std::ios::in);

    int horizon, num_iters;
    is >> horizon >> num_iters;

    double alpha;
    is >> alpha;

    Dynamics dynamics(50, 0.01, 1, 60);
    for (int i = 0; i < dynamics.params.x.size(); ++i)
        is >> dynamics.params.x(i);
    is >> dynamics.params.u(0) >> dynamics.params.u(1);

    Eigen::VectorXd scale_state(8);
    for (int i = 0; i < scale_state.size(); ++i)
        is >> scale_state(i);

    Eigen::Vector2d scale_action;
    is >> scale_action(0) >> scale_action(1);

    Cost cost(alpha, scale_state, scale_action);
    for (int i = 0; i < cost.params.x_star.size(); ++i)
        is >> cost.params.x_star(i);

    LQR lqr(horizon, 0.01, 2, dynamics, cost);
    std::cout << "initialization finished" << std::endl;

    {
        lqr.nominal_rollout();
        std::fstream os("out.txt", std::ios::out | std::ios::trunc);
        lqr.print(os);
    }

    for (int i = 0; i < num_iters; ++i) {
        std::cout << "iter " << i << ":\t";

        lqr.solve();
        lqr.rollout();

        std::cout << lqr.cost_sum(0) << std::endl;

        std::fstream os("out.txt", std::ios::out | std::ios::trunc);
        lqr.print(os);
    }

    return 0;
}
