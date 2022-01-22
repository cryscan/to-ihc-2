//
// Created by cryscan on 10/28/21.
//

#include "biped_model.h"

namespace Biped {
    Model::Model(int num_iters) :
            Base(num_iters),
            inverse_dynamics(inertia_properties, motion_transforms),
            jsim(inertia_properties, force_transforms) {}

    Model::ContactVector Model::end_effector_positions() const {
        auto q = state.position().joint_position();
        auto base = state.position().base_pose();

        Vector3 L_foot = base * Affine3(homogeneous_transforms.fr_trunk_X_L_foot(q)).translation();
        Vector3 R_foot = base * Affine3(homogeneous_transforms.fr_trunk_X_R_foot(q)).translation();

        ContactVector percussion;
        percussion << L_foot, R_foot;
        return percussion;
    }

    Model::Vector3 Model::com() const {
        auto q = state.position().joint_position();
        auto base = state.position().base_pose();
        return base * rcg::getWholeBodyCOM(inertia_properties, q, homogeneous_transforms);
    }

    Model::Inertia Model::inertia_matrix() const {
        auto q = state.position().joint_position();
        return jsim(q).base();
    }

    Model::Inertia Model::inverse_inertia_matrix() const {
        auto q = state.position().joint_position();
        return Traits<Scalar>::inverse(jsim(q));
    }

    Model::Acceleration Model::nonlinear_terms() const {
        rcg::JointState q = state.position().joint_position();
        rcg::JointState u = state.velocity().joint_velocity();

        // base velocity and gravity should be measured in base frame
        rcg::Velocity v = state.velocity().base_spatial();

        rcg::Acceleration g = rcg::Acceleration::Zero();
        auto r = state.position().base_rotation();
        g.tail<3>() = Traits<Scalar>::inverse(r) * Vector3(0, 0, -rcg::g);

        Acceleration nle;
        rcg::Force f;
        rcg::JointState tau;

        inverse_dynamics.id_fully_actuated(f, tau, g, v, rcg::Acceleration::Zero(), q, u, rcg::JointState::Zero());
        nle << f, tau;
        return nle;
    }

    std::tuple<Model::JointState, Model::Vector6>
    Model::id(const Model::JointState& qdd, const ContactVector& f) const {
        rcg::JointState q = state.position().joint_position();
        rcg::JointState u = state.velocity().joint_velocity();

        rcg::Velocity v = state.velocity().base_spatial();

        rcg::Acceleration g = rcg::Acceleration::Zero();
        auto r = state.position().base_rotation();
        g.tail<3>() = Traits<Scalar>::inverse(r) * Vector3(0, 0, -rcg::g);

        decltype(inverse_dynamics)::ExtForces ext_forces;
        {
            rcg::Force t;
            Vector3 p = Affine3(homogeneous_transforms.fr_trunk_X_L_foot(q)).translation();
            Vector3 l = f.segment<3>(L_FOOT * 3);
            t << p.cross(l), l;
            t = force_transforms.fr_trunk_X_fr_L_shin(q) * t;
            ext_forces[rcg::L_SHIN] = t;
        }
        {
            rcg::Force t;
            Vector3 p = Affine3(homogeneous_transforms.fr_trunk_X_R_foot(q)).translation();
            Vector3 l = f.segment<3>(R_FOOT * 3);
            t << p.cross(l), l;
            t = force_transforms.fr_trunk_X_fr_R_shin(q) * t;
            ext_forces[rcg::R_SHIN] = t;
        }

        rcg::Acceleration a;
        rcg::JointState tau;

        inverse_dynamics.id(tau, a, g, v, q, u, qdd, ext_forces);
        return {tau, a};
    }

    Model::ContactJacobian Model::contact_jacobian() const {
        using JacobianJoint = Eigen::Matrix<Scalar, 3, joint_state_dims>;

        auto q = state.position().joint_position();
        ContactJacobian jacobian = ContactJacobian::Zero();
        Matrix3 jacobian_base_linear = Matrix3::Identity();

        {
            // left foot
            auto t = Eigen::Transform<Scalar, 3, Eigen::Affine>(homogeneous_transforms.fr_trunk_X_L_foot(q));
            Vector3 p = t.translation();

            JacobianJoint jacobian_joint = JacobianJoint::Zero();
            jacobian_joint.middleCols<3>(0) = jacobians.fr_trunk_J_L_foot(q).bottomRows<3>();

            Matrix3 jacobian_base_angular = -cross_product(p);
            jacobian.middleRows<3>(L_FOOT * 3) << jacobian_base_angular, jacobian_base_linear, jacobian_joint;
        }
        {
            // right foot
            auto t = Eigen::Transform<Scalar, 3, Eigen::Affine>(homogeneous_transforms.fr_trunk_X_R_foot(q));
            Vector3 p = t.translation();

            JacobianJoint jacobian_joint = JacobianJoint::Zero();
            jacobian_joint.middleCols<3>(3) = jacobians.fr_trunk_J_R_foot(q).bottomRows<3>();

            Matrix3 jacobian_base_angular = -cross_product(p);
            jacobian.middleRows<3>(R_FOOT * 3) << jacobian_base_angular, jacobian_base_linear, jacobian_joint;
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
}