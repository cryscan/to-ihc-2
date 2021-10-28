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

using Biped::rcg::ScalarTraits;

int main() {
    rbd::State<double, 6> state;

    state.velocity().base_angular() << 0, 0, M_PI_2;
    state.position() += state.velocity();

    std::cout << state.transpose();
}