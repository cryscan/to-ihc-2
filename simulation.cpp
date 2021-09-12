//
// Created by cryscan on 9/11/21.
//

#include <vector>
#include <fstream>

#include "common.h"
#include "kinetics.h"
#include "dynamics.h"

int main() {
    Kinetics kinetics("kinetics");
    Dynamics dynamics("dynamics", 50, 0.01, 1, 60);

    kinetics.Base::build_map();
    dynamics.Base::build_map();

    State x0;
    x0 << 1.0, 0.0, 0.785, -1.57, 0.0, 0.0, 0.0, 0.0;

    int horizon = 200;
    std::vector<State> x(horizon + 1, x0);
    std::vector<Action> u(horizon, Action::Zero());

    for (int i = 0; i < horizon; ++i) {
        kinetics.params.x = x[i];
        kinetics.Base::evaluate();

        dynamics.params.x = x[i];
        dynamics.params.u = u[i];
        dynamics.params.d = kinetics.get_foot_pos().z();
        dynamics.Base::evaluate();

        x[i + 1] = dynamics.get_f();
    }

    std::fstream os("out.txt", std::ios::out | std::ios::trunc);
    for (int i = 0; i < horizon; ++i)
        os << x[i].transpose() << '\t' << u[i].transpose() << std::endl;
}