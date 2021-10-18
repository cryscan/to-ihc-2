//
// Created by cryscan on 9/11/21.
//

#include <memory>
#include <vector>
#include <fstream>

#include "common.h"
#include "kinetics.h"
#include "dynamics.h"
#include "cost.h"
#include "lqr.h"
#include "slip.h"

std::unique_ptr<Kinetics> kinetics;
std::unique_ptr<Dynamics> dynamics;
std::unique_ptr<SLIP> slip;

extern "C" {
void init() {
    kinetics = std::make_unique<Kinetics>("kinetics");
    dynamics = std::make_unique<Dynamics>("dynamics", 50, 0.01, 1, 60);

    kinetics->Base::build_map();
    dynamics->Base::build_map();
}

/*
void step(double* in_x, double* in_u, double* out_x) {
    Eigen::Map<State> x(in_x);
    Eigen::Map<Action> u(in_u);

    kinetics->params.x << x;
    kinetics->Base::evaluate();

    dynamics->params.x << x;
    dynamics->params.u << u;
    dynamics->params.d << kinetics->get_body_pos().z(),
            kinetics->get_knee_pos().z(),
            kinetics->get_foot_pos().z();
    dynamics->Base::evaluate();

    Eigen::Map<State>(out_x) << dynamics->get_f();
}
 */

void slip_init(double dt, double mass, double l0, double k) {
    slip = std::make_unique<SLIP>(dt, mass, l0, k);
}

void slip_set_theta(double theta) { slip->set_theta(theta); }
void slip_set_state(SLIP::State* state) { slip->set_state(*state); }
void slip_step() { slip->step(); }

void slip_get_state(SLIP::State* state) { *state = slip->get_state(); }
bool slip_get_stance() { return slip->get_stance(); }

double slip_contact_position() { return slip->contact_position(); }
double slip_apex_height() { return slip->apex_height(); }
double slip_apex_velocity() { return slip->apex_velocity(); }
}

int main() {
    init();

    State x0;
    x0 << 0.5, 0.0, 0.785, -1.57, 0.0, 0.0, 0.0, 0.0;

    int horizon = 200;
    std::vector<State> x(horizon + 1, x0);
    std::vector<Action> u(horizon, Action::Zero());

    for (int i = 0; i < horizon; ++i) {
        kinetics->params.x = x[i];
        kinetics->Base::evaluate();

        dynamics->params.x = x[i];
        dynamics->params.u = u[i];
        dynamics->params.d << kinetics->get_body_pos().z(), kinetics->get_knee_pos().z(), kinetics->get_foot_pos().z();
        dynamics->Base::evaluate();

        x[i + 1] = dynamics->get_f();
    }

    std::fstream os("out.txt", std::ios::out | std::ios::trunc);
    for (int i = 0; i < horizon; ++i)
        os << x[i].transpose() << '\t' << u[i].transpose() << std::endl;
}