#include <iostream>
#include <fstream>

#include "dynamics.h"
#include "cost.h"

using namespace Hopper;

int main() {
    std::fstream fs("out.txt", std::ios::out | std::ios::trunc);
    std::fstream in("in.txt", std::ios::in);

    int num_iters;
    in >> num_iters;

    Dynamics dynamics(50, 0.01, 1);
    dynamics.build_map();

    DynamicsParams dynamics_params;
    in >> dynamics_params.x(0) >> dynamics_params.x(1) >> dynamics_params.x(2) >> dynamics_params.x(3);
    in >> dynamics_params.x(4) >> dynamics_params.x(5) >> dynamics_params.x(6) >> dynamics_params.x(7);
    in >> dynamics_params.u(0) >> dynamics_params.u(1);

    Eigen::VectorXd scale_state(8);
    scale_state << 1.0, 0.1, 0.01, 0.01, 1.0, 1.0, 1.0, 1.0;
    Eigen::Vector2d scale_action(0.1, 0.1);

    Cost cost(0.9, scale_state, scale_action);
    cost.build_map();

    CostParams cost_params;
    cost_params.x_star << -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    for (int i = 0; i < num_iters; ++i) {
        dynamics.evaluate(dynamics_params);

        Eigen::Vector3d foot_pos = dynamics.get_foot_pos();
        dynamics_params.active = foot_pos(2) < 0;

        dynamics_params.x = dynamics.get_f();
        dynamics.print(fs, dynamics_params);

        cost_params.x = dynamics_params.x;
        cost_params.u = dynamics_params.u;
        cost_params.x_ = cost_params.x;
        cost_params.u_ = cost_params.u;

        cost.evaluate(cost_params);
    }

    return 0;
}
