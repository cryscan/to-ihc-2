//
// Created by cryscan on 10/11/21.
//

#include <cmath>

#include "common.h"
#include "slip.h"

SLIP::SLIP(double dt, double mass, double l0, double k) :
        dt(dt),
        mass(mass),
        l0(l0),
        k(k),
        theta(0),
        stance(false),
        x0(0) {}

void SLIP::set_theta(double t) {
    theta = t;
}

void SLIP::set_state(const State& state) {
    q << state.x, state.z;
    u << state.dx, state.dz;

    double l = state.z / std::cos(theta);
    stance = (l < l0);
}

SLIP::State SLIP::get_state() const {
    return {q(0), q(1), u(0), u(1)};
}

void SLIP::step() {
    using Eigen::Vector2d;

    static const auto m = mass;
    static const auto g = static_cast<double>(Robot::rcg::g);

    Vector2d qm = q + u * dt / 2;
    Vector2d h;

    if (stance) {
        double x = qm(0);
        double z = qm(1);
        double l = std::sqrt((x - x0) * (x - x0) + z * z);

        h(0) = k * (x - x0) * (l0 - l) / l;
        h(1) = k * z * (l0 - l) / l - m * g;
    } else
        h << 0, -m * g;

    Vector2d ue = u + h * dt / m;
    Vector2d qe = q + (u + ue) * dt / 2;

    // state transition
    double x = qe(0);
    double z = qe(1);

    if (stance) {
        double l = std::sqrt((x - x0) * (x - x0) + z * z);
        if (l > l0) stance = false;
    } else {
        double l = z / std::cos(theta);
        if (l < l0) {
            stance = true;
            x0 = x + l0 * std::sin(theta);
        }
    }

    q = qe;
    u = ue;
}

double SLIP::apex_height() const {
    if (stance) return 0;

    static const auto g = static_cast<double>(Robot::rcg::g);
    return q(1) + u(1) * u(1) / g / 2;
}
double SLIP::apex_velocity() const {
    if (stance) return 0;
    return u(0);
}
