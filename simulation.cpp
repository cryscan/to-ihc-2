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
#include "gen/contact_forces.h"
#include "gen/regulator.h"

using Biped::rcg::ScalarTraits;

template<typename Scalar>
void set_init_position(rbd::Position<Scalar, Biped::rcg::JointSpaceDimension>& position) {
    typename rbd::Position<Scalar, Biped::rcg::JointSpaceDimension>::Velocity delta;
    delta.base_angular() << M_PI, 0, 0;

    // position.base_position() << 0, 0, 0.725;
    position.base_position() << 0, 0, 1;
    // position.base_position() << 0, 0, -0.724;
    // position += delta;

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

    gen::ContactForces<Biped::Model> contact_forces(model);
    contact_forces.build_map();

    gen::Regulator<Biped::Model> regulator(model);
    regulator.build_map();

    set_init_position(dynamics.params.x.position());
    dynamics.params.u.setZero();

    regulator.params.kp = 2500;
    regulator.params.kv = 100;
    regulator.params.ka = 1;
    regulator.params.qd << 0, M_PI_4, -M_PI_2, 0, M_PI_4, -M_PI_2;
    regulator.params.vd.setZero();
    regulator.params.ad.setZero();

    std::ofstream osx("force.txt");

    std::ofstream os("out.txt");
    os << dynamics.params.x.transpose() << '\n';

    for (int i = 0; i < 1000; ++i) {
        kinematics.params.x << dynamics.params.x;
        kinematics.evaluate();

        for (int j = 0, k = 2; j < Biped::Model::num_contacts; ++j, k += 3)
            dynamics.params.d(j) = kinematics.f(k);

        inertia.params.x << dynamics.params.x;
        inertia.evaluate();

        nonlinear_terms.params.x << dynamics.params.x;
        nonlinear_terms.evaluate();

        constraints.params.x << dynamics.params.x;
        constraints.evaluate();

        contact_forces.params.x << dynamics.params.x;
        contact_forces.params.u << dynamics.params.u;
        contact_forces.params.d << dynamics.params.d;
        contact_forces.evaluate();

        regulator.params.x << dynamics.params.x;
        regulator.params.f << 0.5 * contact_forces.f;
        regulator.evaluate();

        osx << i << ".\t" << contact_forces.f.transpose() << '\n';
        osx << i << ".\t" << regulator.f.transpose() << '\n';

        dynamics.params.u << regulator.f;
        dynamics.evaluate();

        dynamics.params.x << dynamics.f;

        os << dynamics.f.transpose() << '\n';
    }
}