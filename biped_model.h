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
            Model,
            rcg::ScalarTraits::Scalar,
            rcg::JointSpaceDimension,
            rcg::JointSpaceDimension,
            2> {
    public:
        Model();

        ContactVector end_effector_positions() const override;

        Inertia inverse_inertia_matrix() const override;
        AccelerationType nonlinear_terms() const override;
        ContactJacobian contact_jacobian() const override;

    private:
        // kinematics
        mutable rcg::MotionTransforms motion_transforms;
        mutable rcg::ForceTransforms force_transforms;
        mutable rcg::HomogeneousTransforms homogeneous_transforms;
        mutable rcg::Jacobians jacobians;

        // dynamics
        mutable rcg::InertiaProperties inertia_properties;
        mutable rcg::InverseDynamics inverse_dynamics;
        mutable rcg::JSIM jsim;

        static Matrix3 cross_product(const Vector3& p);
    };
}

#endif //TO_IHC_2_BIPED_MODEL_H
