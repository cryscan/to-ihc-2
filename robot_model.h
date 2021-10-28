//
// Created by cryscan on 10/28/21.
//

#ifndef TO_IHC_2_ROBOT_MODEL_H
#define TO_IHC_2_ROBOT_MODEL_H

#include <type_traits>

#include "rbd_state.h"

namespace rbd {
    template<typename Derived, typename Scalar, int JointStateDims, int ControlDims, int NumContacts>
    class ModelBase {
    public:
        using StateType = State<Scalar, JointStateDims>;
        using PositionType = typename StateType::PositionType;
        using VelocityType = typename StateType::VelocityType;
        using AccelerationType = typename StateType::VelocityType;

        static constexpr int joint_state_dims = JointStateDims;
        static constexpr int position_dims = StateType::position_dims;
        static constexpr int velocity_dims = StateType::velocity_dims;
        static constexpr int state_dims = position_dims + velocity_dims;

        static constexpr int control_dims = ControlDims;

        static constexpr int num_contacts = NumContacts;
        static constexpr int contact_dims = 3 * num_contacts;

        using JointState = Eigen::Matrix<Scalar, joint_state_dims, 1>;

        using Inertia = Eigen::Matrix<Scalar, velocity_dims, velocity_dims>;
        using ContactJacobian = Eigen::Matrix<Scalar, contact_dims, velocity_dims>;
        using ContactJacobianTranspose = Eigen::Matrix<Scalar, velocity_dims, contact_dims>;
        using ContactInertia = Eigen::Matrix<Scalar, contact_dims, contact_dims>;
        using ContactVector = Eigen::Matrix<Scalar, contact_dims, 1>;

        using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
        using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

        virtual ContactVector end_effector_positions() const = 0;

        virtual Inertia inverse_inertia_matrix() const = 0;
        virtual AccelerationType nonlinear_terms() const = 0;
        virtual ContactJacobian contact_jacobian() const = 0;

        StateType state;
    };
}

#endif //TO_IHC_2_ROBOT_MODEL_H
