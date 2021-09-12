//
// Created by cryscan on 8/17/21.
//

#include <iostream>
#include <Eigen/Geometry>

#include "dynamics.h"

using namespace Hopper;

namespace {
    using Scalar = Dynamics::Scalar;
    using ScalarTraits = Dynamics::ScalarTraits;

    inline Scalar min(const Scalar& a, const Scalar& b) { return ScalarTraits::min(a, b); }
    inline Scalar max(const Scalar& a, const Scalar& b) { return ScalarTraits::max(a, b); }
}

Dynamics::Dynamics(int num_iters, double dt, double mu, double torque_limit) :
        inverse_dynamics(inertia_properties, motion_transforms),
        jsim(inertia_properties, force_transforms),
        num_iters(num_iters),
        dt(dt),
        mu(mu),
        torque_limit(torque_limit),
        ad_torque_limit(torque_limit) {}

std::tuple<Dynamics::JointState, Dynamics::JointState> Dynamics::step() const {
    using ContactJacobian = Hopper::rcg::Matrix<3, joint_space_dims>;
    using ContactJacobianTranspose = Hopper::rcg::Matrix<joint_space_dims, 3>;

    JointState qm = q + u * dt / 2.0;

    JointState h, nle;
    h << 0, 0, tau.unaryExpr([this](auto x) { return std::min(ad_torque_limit, std::max(-ad_torque_limit, x)); });
    inverse_dynamics.id(nle, qm, u, JointState::Zero());
    h -= nle;

    jsim.update(qm);
    jsim.computeL();
    jsim.computeInverse();

    ContactJacobian J = jacobians.fr_u0_J_foot(qm).bottomRows<3>();
    JointState m_h = jsim.getInverse() * h;
    ContactJacobianTranspose m_Jt = jsim.getInverse() * J.transpose();

    Vector3 p = Vector3::Zero();

    Matrix3 G = J * m_Jt;
    Vector3 c = J * u + J * m_h * dt;

    G += Matrix3::Identity() * 0.1 * ScalarTraits::exp(12 * ScalarTraits::tanh(20 * d));
    c += Vector3(0.0, 0.0, min(d, Scalar(0)) / dt);

    Scalar r = 0.1;
    p << 0, 0, inertia_properties.getTotalMass() * rcg::g * dt;
    for (int i = 0; i < num_iters; ++i)
        p = prox(p - r * (G * p + c));

    JointState qe, ue;
    ue = u + m_h * dt + m_Jt * p;
    qe = q + (u + ue) * dt / 2.0;

    return {qe, ue};
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
    d = ad_x(it);

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

    y = ad_fun.Forward(0, x);
    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(f, y, it, state_dims)

    if (option == EvalOption::FIRST_ORDER) {
        Jacobian jac(output_dims, input_dims);
        JACOBIAN_VIEW(jac) = ad_fun.Jacobian(x);
        it = 0;
        jac = jac.topRows<state_dims>();
        ASSIGN_COLS(df_dx, jac, it, state_dims)
        ASSIGN_COLS(df_du, jac, it, action_dims)
    }
}
