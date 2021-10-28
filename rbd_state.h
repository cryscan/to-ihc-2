//
// Created by cryscan on 10/27/21.
//

#ifndef TO_IHC_2_RBD_STATE_H
#define TO_IHC_2_RBD_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#define SEGMENT(name, length, offset) \
auto& (name)() {                      \
    using Vector = Eigen::Matrix<Scalar, (length), 1, Eigen::DontAlign>; \
    return *reinterpret_cast<Vector*>(this->data() + (offset));          \
}                                     \
                                      \
const auto& (name)() const {          \
    using Vector = Eigen::Matrix<Scalar, (length), 1, Eigen::DontAlign>; \
    return *reinterpret_cast<Vector const*>(this->data() + (offset));    \
}

#define SEGMENT_TYPE(name, type, offset) \
auto& (name)() {                         \
    return *reinterpret_cast<type*>(this->data() + (offset)); \
}                                        \
                                         \
const auto& (name)() const {             \
    return *reinterpret_cast<type const*>(this->data() + (offset)); \
}

namespace rbd {
    template<typename Scalar, int JointSpaceDims>
    struct Position : public Eigen::Matrix<Scalar, 7 + JointSpaceDims, 1> {
        static constexpr int joint_space_dims = JointSpaceDims;
        static constexpr int position_dims = 7 + JointSpaceDims;
        static constexpr int velocity_dims = 6 + JointSpaceDims;

        using Base = Eigen::Matrix<Scalar, position_dims, 1>;

        struct Velocity : public Eigen::Matrix<Scalar, velocity_dims, 1> {
            using Base = Eigen::Matrix<Scalar, velocity_dims, 1>;

            Velocity() { this->setZero(); }

            SEGMENT(base_spatial, 6, 0)
            SEGMENT(base_angular, 3, 0)
            SEGMENT(base_linear, 3, 3)
            SEGMENT(joint_velocity, joint_space_dims, 6)

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

        auto& operator+=(const Velocity& velocity) {
            this->template tail<3 + joint_space_dims>() += velocity.template tail<3 + joint_space_dims>();

            auto rotation = base_rotation();
            rotation = velocity.base_angle_axis() * rotation;
            this->template head<4>() << rotation.normalized().coeffs();

            return *this;
        }

        auto operator+(const Velocity& velocity) const {
            Position<Scalar, joint_space_dims> temp = *this;
            return temp += velocity;
        }

        SEGMENT_TYPE(base_rotation, Eigen::Quaternion<Scalar>, 0)
        SEGMENT(base_position, 3, 4)
        SEGMENT(joint_position, joint_space_dims, 7)

        auto base_pose() const {
            Eigen::Transform<Scalar, 3, Eigen::Affine> transform(base_rotation());
            transform.translation() = base_position();
            return transform;
        }
    };

    template<typename Scalar, int JointSpaceDims>
    class State : public Eigen::Matrix<Scalar, 13 + 2 * JointSpaceDims, 1> {
    public:
        using PositionType = Position<Scalar, JointSpaceDims>;
        using VelocityType = typename PositionType::Velocity;

        static constexpr int joint_space_dims = JointSpaceDims;
        static constexpr int position_dims = PositionType::position_dims;
        static constexpr int velocity_dims = PositionType::velocity_dims;

        using Base = Eigen::Matrix<Scalar, position_dims + velocity_dims, 1>;

        State() {
            position() = PositionType();
            velocity() = VelocityType();
        }

        SEGMENT_TYPE(position, PositionType, 0)
        SEGMENT_TYPE(velocity, VelocityType, position_dims)
    };
}

#undef SEGMENT
#undef SEGMENT_TYPE

#endif //TO_IHC_2_RBD_STATE_H
