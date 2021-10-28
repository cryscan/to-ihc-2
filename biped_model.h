//
// Created by cryscan on 10/28/21.
//

#ifndef TO_IHC_2_BIPED_MODEL_H
#define TO_IHC_2_BIPED_MODEL_H

#include "biped/declarations.h"
#include "biped/rbd_types.h"
#include "biped/transforms.h"
#include "biped/jacobians.h"
#include "biped/inertia_properties.h"
#include "biped/inverse_dynamics.h"
#include "biped/jsim.h"

#include "robot_model.h"

namespace Biped {
    class Model : public rbd::ModelBase<
            Biped::rcg::ScalarTraits::Scalar,
            Biped::rcg::JointSpaceDimension,
            Biped::rcg::JointSpaceDimension,
            2> {
    public:
        Model();

        Percussion end_effector_positions() const override;

        Inertia inverse_inertia_matrix() const override;

    private:
        // kinematics
        mutable Biped::rcg::MotionTransforms motion_transforms;
        mutable Biped::rcg::ForceTransforms force_transforms;
        mutable Biped::rcg::HomogeneousTransforms homogeneous_transforms;
        mutable Biped::rcg::Jacobians jacobians;

        // dynamics
        mutable Biped::rcg::InertiaProperties inertia_properties;
        mutable Biped::rcg::InverseDynamics inverse_dynamics;
        mutable Biped::rcg::JSIM jsim;
    };
}

#endif //TO_IHC_2_BIPED_MODEL_H
