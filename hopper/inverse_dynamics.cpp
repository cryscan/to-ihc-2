#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"

#ifndef EIGEN_NO_DEBUG
#include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace Hopper::rcg;

// Initialization of static-const data
const Hopper::rcg::InverseDynamics::ExtForces
        Hopper::rcg::InverseDynamics::zeroExtForces(Force::Zero());

Hopper::rcg::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
        inertiaProps(&inertia),
        xm(&transforms),
        u1_I(inertiaProps->getTensor_u1()),
        u2_I(inertiaProps->getTensor_u2()),
        body_I(inertiaProps->getTensor_body()),
        leg_I(inertiaProps->getTensor_leg()) {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot Hopper, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    u1_v.setZero();
    u2_v.setZero();
    body_v.setZero();
    leg_v.setZero();

    vcross.setZero();
}

void Hopper::rcg::InverseDynamics::id(
        JointState& jForces,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext) {
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

void Hopper::rcg::InverseDynamics::G_terms(JointState& jForces) {
    // Link 'u1'
    u1_a = (xm->fr_u1_X_fr_u0).col(iit::rbd::LZ) * Hopper::rcg::g;
    u1_f = u1_I * u1_a;
    // Link 'u2'
    u2_a = (xm->fr_u2_X_fr_u1) * u1_a;
    u2_f = u2_I * u2_a;
    // Link 'body'
    body_a = (xm->fr_body_X_fr_u2) * u2_a;
    body_f = body_I * body_a;
    // Link 'leg'
    leg_a = (xm->fr_leg_X_fr_body) * body_a;
    leg_f = leg_I * leg_a;

    secondPass(jForces);
}

void Hopper::rcg::InverseDynamics::C_terms(JointState& jForces, const JointState& qd) {
    // Link 'u1'
    u1_v(iit::rbd::LZ) = qd(BH);   // u1_v = vJ, for the first link of a fixed base robot

    // The first joint is prismatic, no centripetal terms.
    u1_f.setZero();

    // Link 'u2'
    u2_v = ((xm->fr_u2_X_fr_u1) * u1_v);
    u2_v(iit::rbd::LZ) += qd(BX);

    motionCrossProductMx<Scalar>(u2_v, vcross);

    u2_a = (vcross.col(iit::rbd::LZ) * qd(BX));

    u2_f = u2_I * u2_a + vxIv(u2_v, u2_I);

    // Link 'body'
    body_v = ((xm->fr_body_X_fr_u2) * u2_v);
    body_v(iit::rbd::AZ) += qd(HFE);

    motionCrossProductMx<Scalar>(body_v, vcross);

    body_a = (xm->fr_body_X_fr_u2) * u2_a + vcross.col(iit::rbd::AZ) * qd(HFE);

    body_f = body_I * body_a + vxIv(body_v, body_I);

    // Link 'leg'
    leg_v = ((xm->fr_leg_X_fr_body) * body_v);
    leg_v(iit::rbd::AZ) += qd(KFE);

    motionCrossProductMx<Scalar>(leg_v, vcross);

    leg_a = (xm->fr_leg_X_fr_body) * body_a + vcross.col(iit::rbd::AZ) * qd(KFE);

    leg_f = leg_I * leg_a + vxIv(leg_v, leg_I);


    secondPass(jForces);
}


void Hopper::rcg::InverseDynamics::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext) {
    // First pass, link 'u1'
    u1_a = (xm->fr_u1_X_fr_u0).col(iit::rbd::LZ) * Hopper::rcg::g;
    u1_a(iit::rbd::LZ) += qdd(BH);
    u1_v(iit::rbd::LZ) = qd(BH);   // u1_v = vJ, for the first link of a fixed base robot

    // The first joint is prismatic, no centripetal terms.
    u1_f = u1_I * u1_a - fext[U1];

    // First pass, link 'u2'
    u2_v = ((xm->fr_u2_X_fr_u1) * u1_v);
    u2_v(iit::rbd::LZ) += qd(BX);

    motionCrossProductMx<Scalar>(u2_v, vcross);

    u2_a = (xm->fr_u2_X_fr_u1) * u1_a + vcross.col(iit::rbd::LZ) * qd(BX);
    u2_a(iit::rbd::LZ) += qdd(BX);

    u2_f = u2_I * u2_a + vxIv(u2_v, u2_I) - fext[U2];

    // First pass, link 'body'
    body_v = ((xm->fr_body_X_fr_u2) * u2_v);
    body_v(iit::rbd::AZ) += qd(HFE);

    motionCrossProductMx<Scalar>(body_v, vcross);

    body_a = (xm->fr_body_X_fr_u2) * u2_a + vcross.col(iit::rbd::AZ) * qd(HFE);
    body_a(iit::rbd::AZ) += qdd(HFE);

    body_f = body_I * body_a + vxIv(body_v, body_I) - fext[BODY];

    // First pass, link 'leg'
    leg_v = ((xm->fr_leg_X_fr_body) * body_v);
    leg_v(iit::rbd::AZ) += qd(KFE);

    motionCrossProductMx<Scalar>(leg_v, vcross);

    leg_a = (xm->fr_leg_X_fr_body) * body_a + vcross.col(iit::rbd::AZ) * qd(KFE);
    leg_a(iit::rbd::AZ) += qdd(KFE);

    leg_f = leg_I * leg_a + vxIv(leg_v, leg_I) - fext[LEG];

}

void Hopper::rcg::InverseDynamics::secondPass(JointState& jForces) {
    // Link 'leg'
    jForces(KFE) = leg_f(iit::rbd::AZ);
    body_f += xm->fr_leg_X_fr_body.transpose() * leg_f;
    // Link 'body'
    jForces(HFE) = body_f(iit::rbd::AZ);
    u2_f += xm->fr_body_X_fr_u2.transpose() * body_f;
    // Link 'u2'
    jForces(BX) = u2_f(iit::rbd::LZ);
    u1_f += xm->fr_u2_X_fr_u1.transpose() * u2_f;
    // Link 'u1'
    jForces(BH) = u1_f(iit::rbd::LZ);
}
