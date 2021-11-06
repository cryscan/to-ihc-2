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

#include "gen/kinematics.h"
#include "gen/dynamics.h"
#include "gen/inertia.h"
#include "gen/constraints.h"
#include "gen/nonlinear_terms.h"

using Biped::rcg::ScalarTraits;

template<typename Scalar>
void set_init_position(rbd::Position<Scalar, Biped::rcg::JointSpaceDimension>& position) {
    typename rbd::Position<Scalar, Biped::rcg::JointSpaceDimension>::Velocity delta;
    delta.base_angular() << 0, 0, 0;

    position.base_position() << 0, 0, 1.0;
    position += delta;

    position.joint_position() << 0, M_PI_4, -M_PI_2, 0, M_PI_4, -M_PI_2;
}

int main() {
    auto model = std::make_shared<Biped::Model>();

    gen::Kinematics<Biped::Model> kinematics(model);
    kinematics.build_map();

    gen::Dynamics<Biped::Model> dynamics(model);
    dynamics.build_map();

    gen::Inertia<Biped::Model> inertia(model);
    inertia.build_map();

    gen::Constraints<Biped::Model> constraints(model);
    constraints.build_map();

    gen::NonlinearTerms<Biped::Model> nonlinear_terms(model);
    nonlinear_terms.build_map();

    set_init_position(dynamics.params.x.position());
    dynamics.params.u.setZero();

    std::ofstream os("out.txt");
    os << dynamics.params.x.transpose() << '\n';

    for (int i = 0; i < 200; ++i) {
        kinematics.params.x << dynamics.params.x;
        kinematics.evaluate();

        inertia.params.x << dynamics.params.x;
        inertia.evaluate();

        constraints.params.x << dynamics.params.x;
        constraints.evaluate();

        nonlinear_terms.params.x << dynamics.params.x;
        nonlinear_terms.evaluate();

        for (int j = 0, k = 2; j < Biped::Model::num_contacts; ++j, k += 3)
            dynamics.params.d(j) = kinematics.f(k);

        dynamics.evaluate();

        dynamics.params.x << dynamics.f;

        os << dynamics.f.transpose() << '\n';
    }
}