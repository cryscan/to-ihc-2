//
// Created by cryscan on 10/28/21.
//

#include "biped_model.h"

namespace Biped {
    namespace {
        using rcg::ScalarTraits;
        using Scalar = ScalarTraits::Scalar;
    }

    Model::Model() :
            inverse_dynamics(inertia_properties, motion_transforms),
            jsim(inertia_properties, force_transforms) {}

    Model::Percussion Model::end_effector_positions() const {
        using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;

        auto q = state.position().joint_position();
        auto base_pose = state.position().base_pose();

        Vector3 left = base_pose * Affine3(homogeneous_transforms.fr_trunk_X_L_foot(q)).translation();
        Vector3 right = base_pose * Affine3(homogeneous_transforms.fr_trunk_X_R_foot(q)).translation();

        Percussion percussion;
        percussion << left, right;
        return percussion;
    }

    Model::Inertia Model::inverse_inertia_matrix() const {
        auto q = state.position().joint_position();
        jsim.update(q);
        return LLT<ScalarTraits>::inverse(jsim);
    }
}