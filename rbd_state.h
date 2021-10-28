//
// Created by cryscan on 10/27/21.
//

#ifndef TO_IHC_2_RBD_STATE_H
#define TO_IHC_2_RBD_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rbd {
    template<typename Scalar, int JointSpaceDims, int Options = 0>
    struct Position : public Eigen::Matrix<Scalar, 7 + JointSpaceDims, 1, Options> {
        static constexpr int joint_space_dims = JointSpaceDims;
        static constexpr int position_dims = 7 + JointSpaceDims;
        static constexpr int velocity_dims = 6 + JointSpaceDims;

        using Base = Eigen::Matrix<Scalar, position_dims, 1, Options>;

        using BaseRotationType = Eigen::Quaternion<Scalar, Options>;
        using BasePositionType = Eigen::Matrix<Scalar, 3, 1, Options | Eigen::DontAlign>;
        using JointPositionType = Eigen::Matrix<Scalar, joint_space_dims, 1, Options | Eigen::DontAlign>;

        template<int Options_ = 0>
        struct Velocity : public Eigen::Matrix<Scalar, velocity_dims, 1, Options_> {
            using Base = Eigen::Matrix<Scalar, velocity_dims, 1, Options_>;

            using BaseAngularType = Eigen::Matrix<Scalar, 3, 1, Options>;
            using BaseLinearType = Eigen::Matrix<Scalar, 3, 1, Options | Eigen::DontAlign>;
            using JointVelocityType = Eigen::Matrix<Scalar, joint_space_dims, 1, Options | Eigen::DontAlign>;

            Velocity() { this->setZero(); }

            auto& base_angular() { return *reinterpret_cast<BaseAngularType*>(this->data()); }
            auto& base_linear() { return *reinterpret_cast<BaseLinearType*>(this->data() + 3); }
            auto& joint_velocity() { return *reinterpret_cast<JointVelocityType*>(this->data() + 6); }

            const auto& base_angular() const { return *reinterpret_cast<BaseAngularType const*>(this->data()); }
            const auto& base_linear() const { return *reinterpret_cast< BaseLinearType const*>(this->data() + 3); }
            const auto& joint_velocity() const {
                return *reinterpret_cast< JointVelocityType const*>(this->data() + 6);
            }

            auto base_angle_axis() const {
                auto angular = base_angular();
                return Eigen::AngleAxis<Scalar>(angular.norm(), angular.normalized());
            }
        };

        Position() {
            base_rotation().setIdentity();
            base_position().setZero();
            joint_position().setZero();
        }

        template<int Options_ = 0>
        auto& operator+=(const Velocity<Options_>& velocity) {
            this->template tail<3 + joint_space_dims>() += velocity.template tail<3 + joint_space_dims>();

            BaseRotationType rotation = base_rotation();
            rotation = velocity.base_angle_axis() * rotation;
            this->template head<4>() << rotation.normalized().coeffs();

            return *this;
        }

        template<int Options_>
        auto operator+(const Velocity<Options_>& velocity) const {
            Position<Scalar, joint_space_dims> temp = *this;
            return temp += velocity;
        }

        auto& base_rotation() { return *reinterpret_cast<BaseRotationType*>(this->data()); }
        auto& base_position() { return *reinterpret_cast<BasePositionType*>(this->data() + 4); }
        auto& joint_position() { return *reinterpret_cast<JointPositionType*>(this->data() + 7); }

        const auto& base_rotation() const { return *reinterpret_cast<BaseRotationType const*>(this->data()); }
        const auto& base_position() const { return *reinterpret_cast<BasePositionType const*>(this->data() + 4); }
        const auto& joint_position() const { return *reinterpret_cast<JointPositionType const*>(this->data() + 7); }
    };

    template<typename Scalar, int JointSpaceDims, int Options = 0>
    class State : public Eigen::Matrix<Scalar, 13 + 2 * JointSpaceDims, 1> {
    public:
        using PositionType = Position<Scalar, JointSpaceDims, Options>;
        using VelocityType = typename PositionType::template Velocity<Options | Eigen::DontAlign>;

        static constexpr int joint_space_dims = JointSpaceDims;
        static constexpr int position_dims = PositionType::position_dims;
        static constexpr int velocity_dims = PositionType::velocity_dims;

        using Base = Eigen::Matrix<Scalar, position_dims + velocity_dims, 1>;

        State() {
            position() = PositionType();
            velocity() = VelocityType();
        }

        auto& position() { return *reinterpret_cast<PositionType*>(this->data()); }
        auto& velocity() { return *reinterpret_cast<VelocityType*>(this->data() + position_dims); }

        const auto& position() const { return *reinterpret_cast<const PositionType*>(this->data()); }
        const auto& velocity() const { return *reinterpret_cast<const VelocityType*>(this->data() + position_dims); }
    };
}

#endif //TO_IHC_2_RBD_STATE_H
