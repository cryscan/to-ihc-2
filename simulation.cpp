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

int main() {
    /*
    rbd::State<double, 6> state;

    state.velocity().base_angular() << 0, 0, M_PI_2;
    state.velocity().base_linear() << 0, 1, 0;
    state.position() += state.velocity();

    std::cout << state.transpose() << std::endl;
     */

    auto biped_model = new Biped::Model;

    biped_model->control.setZero();
    biped_model->state.position().base_position() << 0, 0, 1;

    {
        Biped::Model::Velocity delta;
        delta(2) = M_PI_2;
        biped_model->state.position() += delta;
    }
//    {
//        Biped::Model::Velocity delta;
//        delta(1) = M_PI_2;
//        biped_model->state.position() += delta;
//    }

    auto positions = biped_model->end_effector_positions();
    std::cout << positions.transpose() << '\n' << std::endl;

    std::cout << biped_model->inverse_inertia_matrix() << '\n' << std::endl;
    std::cout << biped_model->nonlinear_terms().transpose() << '\n' << std::endl;
    std::cout << biped_model->contact_jacobian() << '\n' << std::endl;

    biped_model->d << positions(2), positions(5);

    auto[m_h, m_Jt_p] = biped_model->contact();
    std::cout << m_h.transpose() << '\n' << m_Jt_p.transpose();
}