//
// Created by cryscan on 10/28/21.
//

#include "biped_model.h"

namespace Biped {
    Model::Model() :
            inverse_dynamics(inertia_properties, motion_transforms),
            jsim(inertia_properties, force_transforms) {}

    Model::ContactVector Model::end_effector_positions() const {
        using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;

        auto q = state.position().joint_position();
        auto base_pose = state.position().base_pose();

        Vector3 left = base_pose * Affine3(homogeneous_transforms.fr_trunk_X_L_foot(q)).translation();
        Vector3 right = base_pose * Affine3(homogeneous_transforms.fr_trunk_X_R_foot(q)).translation();

        ContactVector percussion;
        percussion << left, right;
        return percussion;
    }

    Model::Inertia Model::inverse_inertia_matrix() const {
        auto q = state.position().joint_position();
        return LLT<ScalarTraits>::inverse(jsim(q));
    }

    Model::Acceleration Model::nonlinear_terms() const {
        rcg::JointState q = state.position().joint_position();
        rcg::JointState u = state.velocity().joint_velocity();

        // base velocity and gravity should be measured in base frame
        rcg::Velocity v = state.velocity().base_spatial();

        rcg::Acceleration g = rcg::Acceleration::Zero();
        auto r = state.position().base_rotation();
        g.tail<3>() = r.inverse() * Vector3(0, 0, -rcg::g);

        Acceleration nle;
        rcg::Force f;
        rcg::JointState tau;

        inverse_dynamics.id_fully_actuated(f, tau, g, v, rcg::Acceleration::Zero(), q, u, rcg::JointState::Zero());
        nle << f, tau;

        Acceleration h;
        h << -nle;
        h.joint_velocity() << control;

        return h;
    }

    Model::ContactJacobian Model::contact_jacobian() const {
        using JacobianJoint = Eigen::Matrix<Scalar, 3, joint_state_dims>;

        ContactJacobian jacobian = ContactJacobian::Zero();
        auto q = state.position().joint_position();

        {
            auto t = Eigen::Transform<Scalar, 3, Eigen::Affine>(homogeneous_transforms.fr_trunk_X_L_foot(q));
            Vector3 p = t.translation();

            JacobianJoint jacobian_joint = JacobianJoint::Zero();
            jacobian_joint.middleCols<3>(0) = jacobians.fr_trunk_J_L_foot(q).bottomRows<3>();

            Matrix3 jacobian_base_angular = -cross_product(p);
            Matrix3 jacobian_base_linear = Matrix3::Identity();
            jacobian.middleRows<3>(0) << jacobian_base_angular, jacobian_base_linear, jacobian_joint;
        }
        {
            auto t = Eigen::Transform<Scalar, 3, Eigen::Affine>(homogeneous_transforms.fr_trunk_X_R_foot(q));
            Vector3 p = t.translation();

            JacobianJoint jacobian_joint = JacobianJoint::Zero();
            jacobian_joint.middleCols<3>(3) = jacobians.fr_trunk_J_R_foot(q).bottomRows<3>();

            Matrix3 jacobian_base_angular = -cross_product(p);
            Matrix3 jacobian_base_linear = Matrix3::Identity();
            jacobian.middleRows<3>(3) << jacobian_base_angular, jacobian_base_linear, jacobian_joint;
        }

        return jacobian;
    }

    Model::Matrix3 Model::cross_product(const Model::Vector3& p) {
        Matrix3 px = Matrix3::Zero();
        px(0, 1) = -p.z();
        px(0, 2) = p.y();
        px(1, 0) = p.z();
        px(1, 2) = -p.x();
        px(2, 0) = -p.y();
        px(2, 1) = p.x();
        return px;
    }

    std::tuple<Model::Velocity, Model::Acceleration> Model::contact() const {
        Velocity m_h;
        ContactJacobianTranspose m_Jt;
        ContactJacobian J = contact_jacobian();

        auto q = state.position().joint_position();
        auto u = state.velocity().vector();

        {
            ContactJacobianTranspose Jt = J.transpose();
            auto h = nonlinear_terms();

            auto LU = LLT<ScalarTraits>::cholesky(jsim(q));
            m_h << LLT<ScalarTraits>::cholesky_solve(LU, h);
            m_Jt << LLT<ScalarTraits>::cholesky_solve(LU, Jt);
        }

        ContactInertia G = J * m_Jt;
        ContactVector c = J * u + J * m_h * dt;

        Eigen::DenseIndex it = 0;
        for (int i = 0; i < num_contacts; ++i) {
            G.diagonal().segment<3>(it) += Vector3::Ones() * 0.1 * ScalarTraits::exp(8 * ScalarTraits::tanh(20 * d(i)));
            it += 3;
        }

        auto p = solve_percussion(G, c);

        Velocity m_Jt_p;
        m_Jt_p << m_Jt * p;

        return {m_h, m_Jt_p};
    }
}