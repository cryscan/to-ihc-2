#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const Hopper::rcg::ForwardDynamics::ExtForces
        Hopper::rcg::ForwardDynamics::zeroExtForces(Force::Zero());

Hopper::rcg::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
        inertiaProps(&inertia),
        motionTransforms(&transforms) {
    u1_v.setZero();
    u1_c.setZero();
    u2_v.setZero();
    u2_c.setZero();
    body_v.setZero();
    body_c.setZero();
    leg_v.setZero();
    leg_c.setZero();

    vcross.setZero();
    Ia_p.setZero();
    Ia_r.setZero();

}

void Hopper::rcg::ForwardDynamics::fd(
        JointState& qdd,
        const JointState& qd,
        const JointState& tau,
        const ExtForces& fext/* = zeroExtForces */) {

    u1_AI = inertiaProps->getTensor_u1();
    u1_p = -fext[U1];
    u2_AI = inertiaProps->getTensor_u2();
    u2_p = -fext[U2];
    body_AI = inertiaProps->getTensor_body();
    body_p = -fext[BODY];
    leg_AI = inertiaProps->getTensor_leg();
    leg_p = -fext[LEG];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.

    // + Link u1
    //  - The spatial velocity:
    u1_v(LZ) = qd(BH);

    //  - The bias force term:
    // The first joint is prismatic, no bias force term

    // + Link u2
    //  - The spatial velocity:
    u2_v = (motionTransforms->fr_u2_X_fr_u1) * u1_v;
    u2_v(LZ) += qd(BX);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(u2_v, vcross);
    u2_c = vcross.col(LZ) * qd(BX);

    //  - The bias force term:
    u2_p += vxIv(u2_v, u2_AI);

    // + Link body
    //  - The spatial velocity:
    body_v = (motionTransforms->fr_body_X_fr_u2) * u2_v;
    body_v(AZ) += qd(HFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(body_v, vcross);
    body_c = vcross.col(AZ) * qd(HFE);

    //  - The bias force term:
    body_p += vxIv(body_v, body_AI);

    // + Link leg
    //  - The spatial velocity:
    leg_v = (motionTransforms->fr_leg_X_fr_body) * body_v;
    leg_v(AZ) += qd(KFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(leg_v, vcross);
    leg_c = vcross.col(AZ) * qd(KFE);

    //  - The bias force term:
    leg_p += vxIv(leg_v, leg_AI);


    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;

    // + Link leg
    leg_u = tau(KFE) - leg_p(AZ);
    leg_U = leg_AI.col(AZ);
    leg_D = leg_U(AZ);

    compute_Ia_revolute(leg_AI, leg_U, leg_D, Ia_r);  // same as: Ia_r = leg_AI - leg_U/leg_D * leg_U.transpose();
    pa = leg_p + Ia_r * leg_c + leg_U * leg_u / leg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms->fr_leg_X_fr_body, IaB);
    body_AI += IaB;
    body_p += (motionTransforms->fr_leg_X_fr_body).transpose() * pa;

    // + Link body
    body_u = tau(HFE) - body_p(AZ);
    body_U = body_AI.col(AZ);
    body_D = body_U(AZ);

    compute_Ia_revolute(body_AI, body_U, body_D,
                        Ia_r);  // same as: Ia_r = body_AI - body_U/body_D * body_U.transpose();
    pa = body_p + Ia_r * body_c + body_U * body_u / body_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms->fr_body_X_fr_u2, IaB);
    u2_AI += IaB;
    u2_p += (motionTransforms->fr_body_X_fr_u2).transpose() * pa;

    // + Link u2
    u2_u = tau(BX) - u2_p(LZ);
    u2_U = u2_AI.col(LZ);
    u2_D = u2_U(LZ);

    compute_Ia_prismatic(u2_AI, u2_U, u2_D, Ia_p);  // same as: Ia_p = u2_AI - u2_U/u2_D * u2_U.transpose();
    pa = u2_p + Ia_p * u2_c + u2_U * u2_u / u2_D;
    ctransform_Ia_prismatic(Ia_p, motionTransforms->fr_u2_X_fr_u1, IaB);
    u1_AI += IaB;
    u1_p += (motionTransforms->fr_u2_X_fr_u1).transpose() * pa;

    // + Link u1
    u1_u = tau(BH) - u1_p(LZ);
    u1_U = u1_AI.col(LZ);
    u1_D = u1_U(LZ);



    // ---------------------- THIRD PASS ---------------------- //
    u1_a = (motionTransforms->fr_u1_X_fr_u0).col(LZ) * (Hopper::rcg::g);
    qdd(BH) = (u1_u - u1_U.dot(u1_a)) / u1_D;
    u1_a(LZ) += qdd(BH);

    u2_a = (motionTransforms->fr_u2_X_fr_u1) * u1_a + u2_c;
    qdd(BX) = (u2_u - u2_U.dot(u2_a)) / u2_D;
    u2_a(LZ) += qdd(BX);

    body_a = (motionTransforms->fr_body_X_fr_u2) * u2_a + body_c;
    qdd(HFE) = (body_u - body_U.dot(body_a)) / body_D;
    body_a(AZ) += qdd(HFE);

    leg_a = (motionTransforms->fr_leg_X_fr_body) * body_a + leg_c;
    qdd(KFE) = (leg_u - leg_U.dot(leg_a)) / leg_D;
    leg_a(AZ) += qdd(KFE);


}
