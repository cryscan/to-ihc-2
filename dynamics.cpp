//
// Created by cryscan on 8/17/21.
//

#include <iostream>
#include <Eigen/Geometry>

#include "dynamics.h"

using namespace Robot;

namespace {
    using Scalar = Dynamics::Scalar;
    using ScalarTraits = Dynamics::ScalarTraits;

    inline Scalar min(const Scalar& x, const Scalar& y) { return ScalarTraits::min(x, y); }
    inline Scalar max(const Scalar& x, const Scalar& y) { return ScalarTraits::max(x, y); }
}

Dynamics::Dynamics(const std::string& name, int num_iters, double dt, double mu, double torque_limit) :
        Base(name),
        ContactBase(jacobians, inertia_properties),
        inverse_dynamics(inertia_properties, motion_transforms),
        jsim(inertia_properties, force_transforms),
        num_iters(num_iters),
        dt(dt),
        mu(mu),
        torque_limit(torque_limit) {}

std::tuple<Dynamics::JointState, Dynamics::JointState> Dynamics::step() const {
    JointState qm = q + u * dt / 2.0;

    auto[m_h, m_Jt_p] = contact(qm);

    JointState qe, ue;
    ue = u + m_h * dt + m_Jt_p;
    qe = q + (u + ue) * dt / 2.0;

    return {qe, ue};
}

#define VECTOR3(x, i) (x).segment<3>(i)

std::tuple<Dynamics::JointState, Dynamics::JointState>
Dynamics::contact(const JointState& qm) const {
    auto h = nonlinear_terms(qm);
    auto J = contact_jacobian(qm);

    jsim.update(qm);
    jsim.computeL();
    jsim.computeInverse();

    JointState m_h = jsim.getInverse() * h;
    ContactJacobianTranspose m_Jt = jsim.getInverse() * J.transpose();

    ContactInertia G = J * m_Jt;
    Percussion c = J * u + J * m_h * dt;

    Eigen::DenseIndex it = 0;
    for (int i = 0; i < num_contacts; ++i) {
        VECTOR3(G.diagonal(), it) += Vector3::Ones() * 0.1 * ScalarTraits::exp(8 * ScalarTraits::tanh(20 * d(i)));
        // VECTOR3(G.diagonal(), it)(1) = 1.0;
        // c.segment<3>(it) += Vector3(0, 0, min(d(i) / dt, 0));
        it += 3;
    }

    auto p = solve_percussion(G, c, mu, num_iters);

    return {m_h, m_Jt * p};
}

Dynamics::JointState Dynamics::nonlinear_terms(const Dynamics::JointState& qm) const {
    JointState h = JointState::Zero(), nle;
    Action saturated_tau = tau.unaryExpr([this](auto x) { return min(torque_limit, max(-torque_limit, x)); });

    h.tail<action_dims>() = saturated_tau;
    inverse_dynamics.id(nle, qm, u, JointState::Zero());
    h -= nle;

    return h;
}

void Dynamics::build_map() {
    CppAD::Independent(ad_x);

    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(q, ad_x, it, joint_space_dims)
    ASSIGN_VECTOR(u, ad_x, it, joint_space_dims)
    ASSIGN_VECTOR(tau, ad_x, it, action_dims)
    ASSIGN_VECTOR(d, ad_x, it, num_contacts)

    std::tie(q, u) = step();

    it = 0;
    FILL_VECTOR(ad_y, q, it, joint_space_dims)
    FILL_VECTOR(ad_y, u, it, joint_space_dims)

    ad_fun.Dependent(ad_x, ad_y);
    ad_fun.optimize("no_compare_op");
}

#define JACOBIAN_VIEW(jac) Eigen::Map<Eigen::VectorXd>((jac).data(), (jac).size())

void Dynamics::evaluate(const Params& params, EvalOption option) {
    using Jacobian = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    Eigen::VectorXd x(input_dims);
    Eigen::VectorXd y(output_dims);

    x << params.x, params.u, params.d;
    y = model->ForwardZero(x);

    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(f, y, it, state_dims)

    if (option == EvalOption::FIRST_ORDER) {
        Jacobian jacobian(output_dims, input_dims);
        JACOBIAN_VIEW(jacobian) = model->Jacobian(x);
        jacobian = jacobian.topRows<state_dims>();

        it = 0;
        ASSIGN_COLS(df_dx, jacobian, it, state_dims)
        ASSIGN_COLS(df_du, jacobian, it, action_dims)
    }
}

ContactBase::ContactBase(rcg::Jacobians& jacobians, rcg::InertiaProperties& inertia_properties) :
        jacobians(jacobians),
        inertia_properties(inertia_properties) {}

ContactBase::ContactJacobian ContactBase::contact_jacobian(const ContactBase::JointState& q) const {
    ContactJacobian J;

    Eigen::DenseIndex it = 0;
    FILL_ROWS(J, jacobians.fr_u0_J_body(q).bottomRows<3>(), it, 3)
    FILL_ROWS(J, jacobians.fr_u0_J_knee(q).bottomRows<3>(), it, 3)
    FILL_ROWS(J, jacobians.fr_u0_J_foot(q).bottomRows<3>(), it, 3)

    return J;
}

ContactBase::Percussion
ContactBase::solve_percussion(const ContactBase::ContactInertia& G,
                              const ContactBase::Percussion& c,
                              const ContactBase::Scalar& mu,
                              int num_iters) {
    auto remove_nan = [](auto x) { return ScalarTraits::remove_nan(x, 0); };
    ContactInertia LU = ScalarTraits::cholesky(G);
    Percussion p = -ScalarTraits::cholesky_solve(LU, c).unaryExpr(remove_nan);

    Percussion r;
    for (int i = 0; i < contact_dims; i += 3) {
        auto max_abs = [](auto x) { return max(ScalarTraits::abs(x), 1); };
        Scalar det = VECTOR3(LU.diagonal().unaryExpr(max_abs), i).prod();
        VECTOR3(r, i).fill(1 / det / det);
    }

    for (int k = 0; k < num_iters; ++k) {
        p -= r.cwiseProduct(G * p + c);

        for (int i = 0; i < contact_dims; i += 3)
            VECTOR3(p, i) = prox(VECTOR3(p, i), mu);
    }

    return p;
}

ContactBase::Vector3 ContactBase::prox(const ContactBase::Vector3& p,
                                       const ContactBase::Scalar& mu) {
    auto pn = max(Scalar(0), p(2));
    auto px = min(mu * pn, max(-mu * pn, p(0)));
    auto py = min(mu * pn, max(-mu * pn, p(1)));
    return {px, py, pn};
}
