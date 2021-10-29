//
// Created by cryscan on 10/28/21.
//

#ifndef TO_IHC_2_ROBOT_MODEL_H
#define TO_IHC_2_ROBOT_MODEL_H

#include <type_traits>

#include "rbd_state.h"

#define SEGMENT3(x, i) ((x).segment<3>(i))

template<typename Derived,
        typename ScalarTraits_ = typename Derived::ScalarTraits,
        int JointStateDims = Derived::joint_state_dims,
        int ControlDims = Derived::control_dims,
        int NumContacts = Derived::num_contacts>
class ModelBase {
public:
    using ScalarTraits = ScalarTraits_;
    using Scalar = typename ScalarTraits::Scalar;

    using State = rbd::State<Scalar, JointStateDims>;
    using Position = typename State::PositionType;
    using Velocity = typename State::VelocityType;
    using Acceleration = typename State::VelocityType;

    static constexpr int joint_state_dims = JointStateDims;
    static constexpr int position_dims = State::position_dims;
    static constexpr int velocity_dims = State::velocity_dims;
    static constexpr int state_dims = position_dims + velocity_dims;

    static constexpr int control_dims = ControlDims;

    static constexpr int num_contacts = NumContacts;
    static constexpr int contact_dims = 3 * num_contacts;

    static constexpr
    std::remove_reference<ModelBase<Derived, ScalarTraits_, joint_state_dims, contact_dims, num_contacts>>
    base_type() {};

    using Control = Eigen::Matrix<Scalar, contact_dims, 1>;
    using JointState = Eigen::Matrix<Scalar, joint_state_dims, 1>;

    using Inertia = Eigen::Matrix<Scalar, velocity_dims, velocity_dims>;
    using ContactJacobian = Eigen::Matrix<Scalar, contact_dims, velocity_dims>;
    using ContactJacobianTranspose = Eigen::Matrix<Scalar, velocity_dims, contact_dims>;
    using ContactInertia = Eigen::Matrix<Scalar, contact_dims, contact_dims>;
    using ContactVector = Eigen::Matrix<Scalar, contact_dims, 1>;

    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Affine3 = Eigen::Transform<Scalar, 3, Eigen::Affine>;

    virtual ContactVector end_effector_positions() const = 0;

    virtual Inertia inverse_inertia_matrix() const = 0;
    virtual Acceleration nonlinear_terms() const = 0;
    virtual ContactJacobian contact_jacobian() const = 0;

    State state;

protected:
    inline static ContactVector solve_percussion(const Affine3& t,
                                                 const ContactInertia& G,
                                                 const ContactVector& c,
                                                 const Scalar& mu,
                                                 int num_iters) {
        ContactInertia LU = LLT<ScalarTraits>::cholesky(G);
        ContactVector p = -LLT<ScalarTraits>::cholesky_solve(LU, c);

        auto max_abs = [](auto x) { return std::max(ScalarTraits::abs(x), Scalar(1)); };
        ContactVector r;
        ContactVector d = LU.diagonal().unaryExpr(max_abs);
        for (int i = 0; i < contact_dims; i += 3) {
            Scalar det = SEGMENT3(d, i).prod();
            SEGMENT3(r, i).fill(Scalar(1) / det / det);
        }

        for (int k = 0; k < num_iters; ++k) {
            p -= r.template cwiseProduct(G * p + c);
            p = t.linear() * p;

            for (int i = 0; i < contact_dims; i += 3)
                SEGMENT3(p, i) = prox(SEGMENT3(p, i), mu);

            p = t.linear().inverse() * p;
        }

        return p;
    }

    // assume that the normal is pointed at +z axis
    inline static Vector3 prox(const Vector3& p, const Scalar& mu) {
        Scalar pn = std::max(Scalar(0), p(2));
        Scalar n = mu * pn;

        Scalar px = std::min(n, std::max(-n, p(0)));
        Scalar py = std::min(n, std::max(-n, p(1)));

        return {px, py, pn};
    }
};

#undef SEGMENT3

#endif //TO_IHC_2_ROBOT_MODEL_H
