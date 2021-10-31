//
// Created by cryscan on 10/28/21.
//

#ifndef TO_IHC_2_ROBOT_MODEL_H
#define TO_IHC_2_ROBOT_MODEL_H

#include <type_traits>
#include <tuple>

#include "rbd_state.h"

#define SEGMENT3(x, i) ((x).template segment<3>(i))

template<typename Derived,
        typename T = typename Derived::ScalarTraits,
        int JointStateDims = Derived::joint_state_dims,
        int ControlDims = Derived::control_dims,
        int NumContacts = Derived::num_contacts>
class ModelBase {
public:
    using ScalarTraits = T;
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
    std::remove_reference<ModelBase<Derived, T, joint_state_dims, control_dims, num_contacts>>
    base_type() {};

    using Control = Eigen::Matrix<Scalar, control_dims, 1>;
    using JointState = Eigen::Matrix<Scalar, joint_state_dims, 1>;

    using Inertia = Eigen::Matrix<Scalar, velocity_dims, velocity_dims>;
    using ContactJacobian = Eigen::Matrix<Scalar, contact_dims, velocity_dims>;
    using ContactJacobianTranspose = Eigen::Matrix<Scalar, velocity_dims, contact_dims>;
    using ContactInertia = Eigen::Matrix<Scalar, contact_dims, contact_dims>;
    using ContactVector = Eigen::Matrix<Scalar, contact_dims, 1>;
    using ContactDistance = Eigen::Matrix<Scalar, num_contacts, 1>;

    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Quaternion = Eigen::Quaternion<Scalar>;

    virtual ContactVector end_effector_positions() const = 0;

    virtual Inertia inertia_matrix() const = 0;
    virtual Inertia inverse_inertia_matrix() const = 0;
    virtual Acceleration nonlinear_terms() const = 0;
    virtual ContactJacobian contact_jacobian() const = 0;

    inline std::tuple<Acceleration, Velocity> contact() const {
        Velocity m_h, m_Jt_p;
        ContactJacobianTranspose m_Jt;
        ContactJacobian J = contact_jacobian();

        auto q = state.position().joint_position();
        auto u = state.velocity().vector();

        {
            ContactJacobianTranspose Jt = J.transpose();
            auto h = nonlinear_terms();

            auto LU = LLT<ScalarTraits>::cholesky(inertia_matrix());
            m_h = LLT<ScalarTraits>::cholesky_solve(LU, h);
            m_Jt = LLT<ScalarTraits>::cholesky_solve(LU, Jt);
        }

        ContactInertia G = J * m_Jt;
        ContactVector c = J * u + J * m_h * dt;

        Eigen::DenseIndex it = 0;
        for (int i = 0; i < num_contacts; ++i) {
            Vector3 complement = Vector3::Ones() * 0.1 * ScalarTraits::exp(8 * ScalarTraits::tanh(20 * d(i)));
            SEGMENT3(G.diagonal(), it) += complement;

            // Vector3 drift = Vector3::Ones() * d(i) / dt;
            // SEGMENT3(c, it) += drift;

            it += 3;
        }

        auto p = solve_percussion(G, c);
        m_Jt_p = m_Jt * p;

        return {m_h, m_Jt_p};
    }

    const int num_iters = 100;
    const Scalar dt = 0.01;

    State state;
    Control control;

    Scalar mu = 1;
    ContactDistance d;

protected:
    inline ContactVector
    solve_percussion(const ContactInertia& G, const ContactVector& c) const {
        ContactInertia LU = LLT<ScalarTraits>::cholesky(G);
        ContactVector p = -LLT<ScalarTraits>::cholesky_solve(LU, c);

        auto max_abs = [](auto x) { return std::max(ScalarTraits::abs(x), Scalar(1)); };
        ContactVector r;
        ContactVector diagonal = LU.diagonal().unaryExpr(max_abs);
        for (int i = 0; i < contact_dims; i += 3) {
            Scalar det = SEGMENT3(diagonal, i).prod();
            SEGMENT3(r, i).fill(Scalar(1) / det / det);
        }

        auto t = state.position().base_rotation();
        for (int k = 0; k < num_iters; ++k) {
            p -= r.template cwiseProduct(G * p + c);
            for (int i = 0; i < contact_dims; i += 3)
                SEGMENT3(p, i) = t.inverse() * prox(t * SEGMENT3(p, i));
        }

        return p;
    }

    // assume that the normal is pointed at +z axis
    inline Vector3 prox(const Vector3& p) const {
        Scalar pn = std::max(Scalar(0), p(2));
        Scalar n = mu * pn;

        Scalar px = std::min(n, std::max(-n, p(0)));
        Scalar py = std::min(n, std::max(-n, p(1)));

        return {px, py, pn};
    }
};

#undef SEGMENT3

#endif //TO_IHC_2_ROBOT_MODEL_H
