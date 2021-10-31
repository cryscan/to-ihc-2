//
// Created by cryscan on 9/11/21.
//

#include <memory>
#include <vector>
#include <iostream>
#include <fstream>

#include "biped/rbd_types.h"
#include "rbd_state.h"
#include "robot_model.h"
#include "biped_model.h"

using Biped::rcg::ScalarTraits;

template<typename Derived>
void step(ModelBase<Derived>& model) {
    auto q = model.state.position();
    auto u = model.state.velocity();
    auto dt = model.dt;

    auto e = model.end_effector_positions();
    model.d << e(2), e(5), e(8), e(11), e(14), e(17), e(20);

    model.state.position() = q + u * dt / 2;
    auto[m_h, m_Jt_p] = model.contact();

    decltype(u) ue = u + m_h * dt + m_Jt_p;
    model.state.velocity() = ue;
    model.state.position() += (u + ue) * dt / 2;
}

int main() {
    /*
    rbd::State<double, 6> state;

    state.velocity().base_angular() << 0, 0, M_PI_2;
    state.velocity().base_linear() << 0, 1, 0;
    state.position() += state.velocity();

    std::cout << state.transpose() << std::endl;
     */

    auto model = std::make_unique<Biped::Model>();

    model->control.setZero();
    model->state.position().base_position() << 0, 0, 0.7;
    model->state.position().joint_position() << 0, M_PI_4, -M_PI_2, 0, M_PI_4, -M_PI_2;

    std::ofstream os("out.txt");
    os << model->state.transpose() << '\n';

    for (int i = 0; i < 200; ++i) {
        step(*model);
        os << model->state.transpose() << '\n';
    }
}