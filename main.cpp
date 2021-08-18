#include <iostream>
#include <fstream>

#include "dynamics.h"

using namespace Hopper;

int main() {
    std::fstream fs("out.txt", std::ios::out | std::ios::trunc);
    std::fstream in("in.txt", std::ios::in);

    int num_iters;
    in >> num_iters;

    Dynamics dynamics(50, 0.01, 1);

    Dynamics::Params params;
    in >> params.x(0) >> params.x(1) >> params.x(2) >> params.x(3);
    in >> params.x(4) >> params.x(5) >> params.x(6) >> params.x(7);
    in >> params.u(0) >> params.u(1);

    dynamics.build_map();

    for (int i = 0; i < num_iters; ++i) {
        dynamics.evaluate(params);
        Eigen::Vector3d foot_pos = dynamics.get_foot_pos();
        params.active = foot_pos(2) < 0;
        params.x = dynamics.get_f();
        dynamics.print(fs, params);
    }

    return 0;
}
