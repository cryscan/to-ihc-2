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
        inverse_dynamics(inertia_properties, motion_transforms),
        jsim(inertia_properties, force_transforms),
        num_iters(num_iters),
        dt(dt),
        mu(mu),
        torque_limit(torque_limit) {}

std::tuple<Dynamics::JointState, Dynamics::JointState> Dynamics::step() const {
    JointState qm = q + u * dt / 2.0;

    JointState h = JointState::Zero(), nle;
    h.tail<action_dims>() = tau.unaryExpr([this](auto x) { return min(torque_limit, max(-torque_limit, x)); });
    inverse_dynamics.id(nle, qm, u, JointState::Zero());
    h -= nle;

    auto[m_h, m_Jt_p] = contact(qm, h);

    JointState qe, ue;
    ue = u + m_h * dt + m_Jt_p;
    qe = q + (u + ue) * dt / 2.0;

    return {qe, ue};
}

std::tuple<Dynamics::JointState, Dynamics::JointState>
Dynamics::contact(const JointState& qm, const JointState& h) const {
    Eigen::DenseIndex it = 0;

    // stack contact jacobians
    ContactJacobian J;
    FILL_ROWS(J, jacobians.fr_u0_J_body(qm).bottomRows<3>(), it, 3)
    FILL_ROWS(J, jacobians.fr_u0_J_knee(qm).bottomRows<3>(), it, 3)
    FILL_ROWS(J, jacobians.fr_u0_J_foot(qm).bottomRows<3>(), it, 3)

    jsim.update(qm);
    jsim.computeL();
    jsim.computeInverse();

    JointState m_h = jsim.getInverse() * h;
    ContactJacobianTranspose m_Jt = jsim.getInverse() * J.transpose();

    Percussion p = Percussion::Zero();

    ContactInertia G = J * m_Jt;
    Percussion c = J * u + J * m_h * dt;

    it = 0;
    for (int i = 0; i < num_contacts; ++i) {
        Scalar correction = 0.1 * ScalarTraits::exp(8 * ScalarTraits::tanh(20 * d(i)));
        Scalar drift = min(d(i) / dt, 0);

        G.diagonal().segment<3>(it) += Vector3::Ones() * correction;
        c.segment<3>(it) += Vector3(0, 0, drift);

        it += 3;
    }

    Scalar r = 0.1;

    // init percussion
    p = Vector3(0, 0, inertia_properties.getTotalMass() * rcg::g * dt).replicate<num_contacts, 1>();

    for (int k = 0; k < num_iters; ++k) {
        p -= r * (G * p + c);
        for (int i = 0; i < contact_dims; i += 3)
            p.segment<3>(i) = prox(p.segment<3>(i));
    }

    return {m_h, m_Jt * p};
}

Dynamics::Vector3 Dynamics::prox(const Dynamics::Vector3& p) const {
    auto pn = max(Scalar(0), p(2));
    auto px = min(mu * pn, max(-mu * pn, p(0)));
    auto py = min(mu * pn, max(-mu * pn, p(1)));
    return {px, py, pn};
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
        Jacobian jac(output_dims, input_dims);
        JACOBIAN_VIEW(jac) = model->Jacobian(x);
        it = 0;
        jac = jac.topRows<state_dims>();
        ASSIGN_COLS(df_dx, jac, it, state_dims)
        ASSIGN_COLS(df_du, jac, it, action_dims)
    }
}
