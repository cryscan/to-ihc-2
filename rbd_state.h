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
    struct Position : public Eigen::Matrix<Scalar, 7 + JointSpaceDims, 1, Eigen::DontAlign> {
        static constexpr int joint_space_dims = JointSpaceDims;
        static constexpr int position_dims = 7 + JointSpaceDims;
        static constexpr int velocity_dims = 6 + JointSpaceDims;

        struct Velocity : public Eigen::Matrix<Scalar, velocity_dims, 1, Eigen::DontAlign> {
            using Base = Eigen::Matrix<Scalar, velocity_dims, 1, Eigen::DontAlign>;
            Velocity() { this->setZero(); }

            template<typename T>
            Velocity(const T& t) : Base(t) {}

            SEGMENT(vector, velocity_dims, 0)
            SEGMENT(base_spatial, 6, 0)
            SEGMENT(base_angular, 3, 0)
            SEGMENT(base_linear, 3, 3)
            SEGMENT(joint_velocity, joint_space_dims, 6)

            inline auto base_lie() const {
                return Eigen::AngleAxis<Scalar>(base_angular().norm(), base_angular().normalized());
            }
        };

        using Base = Eigen::Matrix<Scalar, 7 + JointSpaceDims, 1, Eigen::DontAlign>;

        Position() {
            base_rotation().setIdentity();
            base_position().setZero();
            joint_position().setZero();
        }

        template<typename T>
        Position(const T& t) : Base(t) {}

        auto& operator+=(const Velocity& velocity) {
            joint_position() += velocity.joint_velocity();

            // velocity expressed in body frame
            auto rotation = base_rotation();
            base_position() += rotation * velocity.base_linear();

            rotation = (rotation * velocity.base_lie()).normalized();
            this->template head<4>() = rotation.coeffs();

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
