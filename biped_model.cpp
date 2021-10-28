//
// Created by cryscan on 10/28/21.
//

#include "biped_model.h"

using Biped::rcg::ScalarTraits;
using Scalar = ScalarTraits::Scalar;

BipedModel::BipedModel() :
        inverse_dynamics(inertia_properties, motion_transforms),
        jsim(inertia_properties, force_transforms) {}

std::array<BipedModel::Vector3, BipedModel::num_contacts> BipedModel::end_effector_positions() const {
    return {};
}
