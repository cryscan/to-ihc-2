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

    Biped::Model biped_model;
    biped_model.state.position().base_position() << 1, 0, 1;

    auto positions = biped_model.end_effector_positions();
    std::cout << positions.transpose() << '\n' << std::endl;

    std::cout << biped_model.inverse_inertia_matrix() << std::endl;
}