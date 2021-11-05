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
    class Model : public ::ModelBase<
            Model,
            rcg::ScalarTraits,
            rcg::JointSpaceDimension,
            rcg::JointSpaceDimension,
            2> {
    public:
        using Base = ModelBase;
        using Base::ScalarTraits;
        using Base::Scalar;

        enum EE {
            L_FOOT, R_FOOT
        };

        explicit Model(int num_iters = 100);

        ContactVector end_effector_positions() const override;

        Inertia inertia_matrix() const override;
        Inertia inverse_inertia_matrix() const override;
        Acceleration nonlinear_terms() const override;
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
