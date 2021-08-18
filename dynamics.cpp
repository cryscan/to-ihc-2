//
// Created by cryscan on 8/17/21.
//

#include "dynamics.h"

using namespace Hopper;

std::tuple<Dynamics::JointState, Dynamics::JointState> Dynamics::step() const {
    JointState qm = q + u * dt / 2.0;

    JointState h, nle;
    h << 0, 0, tau;
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
    Scalar r = 0.1;

    p << 0, 0, inertia_properties.getTotalMass() * rcg::g * dt;
    for (int i = 0; i < num_iters; ++i)
        p = prox(p - r * (G * p + c));

    JointState qe, ue;
    ue = u + m_h * dt + m_Jt * p * active;
    qe = q + (u + ue) * dt / 2.0;
    return {qe, ue};
}

Dynamics::Vector3 Dynamics::prox(const Dynamics::Vector3& p) const {
    auto pn = std::max(Scalar(0), p(2));
    auto px = std::min(mu * pn, std::max(-mu * pn, p(0)));
    auto py = std::min(mu * pn, std::max(-mu * pn, p(1)));
    return {px, py, pn};
}

Dynamics::Vector3 Dynamics::compute_foot_pos() const {
    Affine3 foot(transforms.fr_u0_X_foot(q));
    return foot.translation();
}

void Dynamics::print(std::ostream& os, const Params& params) const {
    JointState q_ = params.x.segment<joint_space_dims>(0).cast<Scalar>();

    Affine3 body(transforms.fr_u0_X_fr_body(q_));
    Affine3 leg(transforms.fr_u0_X_fr_leg(q_));
    Affine3 foot(transforms.fr_u0_X_foot(q_));

    using std::endl;
    os << body.translation().transpose() << '\t'
       << leg.translation().transpose() << '\t'
       << foot.translation().transpose() << '\t' << endl;
}

#define ASSIGN_VECTOR(from, to, it, size) (to) = (from).segment<(size)>(it); (it) += (size);
#define FILL_VECTOR(from, to, it, size) (to).segment<(size)>(it) = (from); (it) += (size);

void Dynamics::evaluate(const Params& params) {
    Eigen::VectorXd x(input_dims);
    Eigen::VectorXd y(output_dims);

    x << params.x, params.u, double(params.active);
    y = ad_fun.Forward(0, x);

    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(y, f, it, state_dims)
    ASSIGN_VECTOR(y, foot_pos, it, 3)
}

void Dynamics::build_map() {
    CppAD::Independent(ad_x);

    Eigen::DenseIndex it = 0;
    ASSIGN_VECTOR(ad_x, q, it, joint_space_dims)
    ASSIGN_VECTOR(ad_x, u, it, joint_space_dims)
    ASSIGN_VECTOR(ad_x, tau, it, action_dims)
    active = ad_x(it);

    std::tie(q, u) = step();

    it = 0;
    FILL_VECTOR(q, ad_y, it, joint_space_dims)
    FILL_VECTOR(u, ad_y, it, joint_space_dims)
    FILL_VECTOR(compute_foot_pos(), ad_y, it, 3)

    ad_fun.Dependent(ad_y);
}