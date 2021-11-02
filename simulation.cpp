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

#include "kinematics.h"
#include "dynamics.h"

using Biped::rcg::ScalarTraits;

template<typename Scalar>
void set_init_position(rbd::Position<Scalar, Biped::rcg::JointSpaceDimension>& position) {
    typename rbd::Position<Scalar, Biped::rcg::JointSpaceDimension>::Velocity delta;
    delta.base_angular() << 0, M_PI_2, 0;

    position.base_position() << 0, 0, 0.7;
    position += delta;

    position.joint_position() << 0, M_PI_4, -M_PI_2, 0, M_PI_4, -M_PI_2;
}

int main() {
    /*
    rbd::State<double, 6> state;

    state.velocity().base_angular() << 0, 0, M_PI_2;
    state.velocity().base_linear() << 0, 1, 0;
    state.position() += state.velocity();

    std::cout << state.transpose() << std::endl;
     */

    auto model = std::make_shared<Biped::Model>();

    Kinematics<Biped::Model> kinematics("kinematics", model);
    kinematics.build_map();

    Dynamics<Biped::Model> dynamics("dynamics", model);
    dynamics.build_map();

    set_init_position(dynamics.params.x.position());
    dynamics.params.x.velocity().base_angular() << M_PI_2, 0, 0;
    dynamics.params.x.velocity().base_linear() << 0, 0, 0;
    dynamics.params.x.velocity().joint_velocity() << 0, 0, M_PI_2, 0, 0, M_PI_2;
    dynamics.params.u.setZero();

    std::ofstream os("out.txt");
    os << dynamics.params.x.transpose() << '\n';

    for (int i = 0; i < 2000; ++i) {
        kinematics.params.q << dynamics.params.x.position();
        kinematics.evaluate();

        for (int j = 0, k = 2; j < Biped::Model::num_contacts; ++j, k += 3)
            dynamics.params.d(j) = kinematics.f(k);

        dynamics.evaluate();

        dynamics.params.x << dynamics.f;

        os << dynamics.f.transpose() << '\n';
    }
}