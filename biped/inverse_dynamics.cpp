#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"

#ifndef EIGEN_NO_DEBUG
#include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace Biped::rcg;

// Initialization of static-const data
const Biped::rcg::InverseDynamics::ExtForces
        Biped::rcg::InverseDynamics::zeroExtForces(Force::Zero());

Biped::rcg::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
        inertiaProps(&inertia),
        xm(&transforms),
        L_hip_I(inertiaProps->getTensor_L_hip()),
        L_thigh_I(inertiaProps->getTensor_L_thigh()),
        L_shin_I(inertiaProps->getTensor_L_shin()),
        R_hip_I(inertiaProps->getTensor_R_hip()),
        R_thigh_I(inertiaProps->getTensor_R_thigh()),
        R_shin_I(inertiaProps->getTensor_R_shin()),
        trunk_I(inertiaProps->getTensor_trunk()),
        L_shin_Ic(L_shin_I),
        R_shin_Ic(R_shin_I) {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot Biped, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    L_hip_v.setZero();
    L_thigh_v.setZero();
    L_shin_v.setZero();
    R_hip_v.setZero();
    R_thigh_v.setZero();
    R_shin_v.setZero();

    vcross.setZero();
}

void Biped::rcg::InverseDynamics::id(
        JointState& jForces, Acceleration& trunk_a,
        const Acceleration& g, const Velocity& trunk_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext) {
    trunk_Ic = trunk_I;
    L_hip_Ic = L_hip_I;
    L_thigh_Ic = L_thigh_I;
    R_hip_Ic = R_hip_I;
    R_thigh_Ic = R_thigh_I;

    // First pass, link 'L_hip'
    L_hip_v = ((xm->fr_L_hip_X_fr_trunk) * trunk_v);
    L_hip_v(iit::rbd::AZ) += qd(L_HAA);

    motionCrossProductMx<Scalar>(L_hip_v, vcross);

    L_hip_a = (vcross.col(iit::rbd::AZ) * qd(L_HAA));
    L_hip_a(iit::rbd::AZ) += qdd(L_HAA);

    L_hip_f = L_hip_I * L_hip_a + vxIv(L_hip_v, L_hip_I);

    // First pass, link 'L_thigh'
    L_thigh_v = ((xm->fr_L_thigh_X_fr_L_hip) * L_hip_v);
    L_thigh_v(iit::rbd::AZ) += qd(L_HFE);

    motionCrossProductMx<Scalar>(L_thigh_v, vcross);

    L_thigh_a = (xm->fr_L_thigh_X_fr_L_hip) * L_hip_a + vcross.col(iit::rbd::AZ) * qd(L_HFE);
    L_thigh_a(iit::rbd::AZ) += qdd(L_HFE);

    L_thigh_f = L_thigh_I * L_thigh_a + vxIv(L_thigh_v, L_thigh_I);

    // First pass, link 'L_shin'
    L_shin_v = ((xm->fr_L_shin_X_fr_L_thigh) * L_thigh_v);
    L_shin_v(iit::rbd::AZ) += qd(L_KFE);

    motionCrossProductMx<Scalar>(L_shin_v, vcross);

    L_shin_a = (xm->fr_L_shin_X_fr_L_thigh) * L_thigh_a + vcross.col(iit::rbd::AZ) * qd(L_KFE);
    L_shin_a(iit::rbd::AZ) += qdd(L_KFE);

    L_shin_f = L_shin_I * L_shin_a + vxIv(L_shin_v, L_shin_I);

    // First pass, link 'R_hip'
    R_hip_v = ((xm->fr_R_hip_X_fr_trunk) * trunk_v);
    R_hip_v(iit::rbd::AZ) += qd(R_HAA);

    motionCrossProductMx<Scalar>(R_hip_v, vcross);

    R_hip_a = (vcross.col(iit::rbd::AZ) * qd(R_HAA));
    R_hip_a(iit::rbd::AZ) += qdd(R_HAA);

    R_hip_f = R_hip_I * R_hip_a + vxIv(R_hip_v, R_hip_I);

    // First pass, link 'R_thigh'
    R_thigh_v = ((xm->fr_R_thigh_X_fr_R_hip) * R_hip_v);
    R_thigh_v(iit::rbd::AZ) += qd(R_HFE);

    motionCrossProductMx<Scalar>(R_thigh_v, vcross);

    R_thigh_a = (xm->fr_R_thigh_X_fr_R_hip) * R_hip_a + vcross.col(iit::rbd::AZ) * qd(R_HFE);
    R_thigh_a(iit::rbd::AZ) += qdd(R_HFE);

    R_thigh_f = R_thigh_I * R_thigh_a + vxIv(R_thigh_v, R_thigh_I);

    // First pass, link 'R_shin'
    R_shin_v = ((xm->fr_R_shin_X_fr_R_thigh) * R_thigh_v);
    R_shin_v(iit::rbd::AZ) += qd(R_KFE);

    motionCrossProductMx<Scalar>(R_shin_v, vcross);

    R_shin_a = (xm->fr_R_shin_X_fr_R_thigh) * R_thigh_a + vcross.col(iit::rbd::AZ) * qd(R_KFE);
    R_shin_a(iit::rbd::AZ) += qdd(R_KFE);

    R_shin_f = R_shin_I * R_shin_a + vxIv(R_shin_v, R_shin_I);

    // The force exerted on the floating base by the links
    trunk_f = vxIv(trunk_v, trunk_I);


    // Add the external forces:
    trunk_f -= fext[TRUNK];
    L_hip_f -= fext[L_HIP];
    L_thigh_f -= fext[L_THIGH];
    L_shin_f -= fext[L_SHIN];
    R_hip_f -= fext[R_HIP];
    R_thigh_f -= fext[R_THIGH];
    R_shin_f -= fext[R_SHIN];

    InertiaMatrix Ic_spare;
    iit::rbd::transformInertia<Scalar>(R_shin_Ic, (xm->fr_R_shin_X_fr_R_thigh).transpose(), Ic_spare);
    R_thigh_Ic += Ic_spare;
    R_thigh_f = R_thigh_f + (xm->fr_R_shin_X_fr_R_thigh).transpose() * R_shin_f;

    iit::rbd::transformInertia<Scalar>(R_thigh_Ic, (xm->fr_R_thigh_X_fr_R_hip).transpose(), Ic_spare);
    R_hip_Ic += Ic_spare;
    R_hip_f = R_hip_f + (xm->fr_R_thigh_X_fr_R_hip).transpose() * R_thigh_f;

    iit::rbd::transformInertia<Scalar>(R_hip_Ic, (xm->fr_R_hip_X_fr_trunk).transpose(), Ic_spare);
    trunk_Ic += Ic_spare;
    trunk_f = trunk_f + (xm->fr_R_hip_X_fr_trunk).transpose() * R_hip_f;

    iit::rbd::transformInertia<Scalar>(L_shin_Ic, (xm->fr_L_shin_X_fr_L_thigh).transpose(), Ic_spare);
    L_thigh_Ic += Ic_spare;
    L_thigh_f = L_thigh_f + (xm->fr_L_shin_X_fr_L_thigh).transpose() * L_shin_f;

    iit::rbd::transformInertia<Scalar>(L_thigh_Ic, (xm->fr_L_thigh_X_fr_L_hip).transpose(), Ic_spare);
    L_hip_Ic += Ic_spare;
    L_hip_f = L_hip_f + (xm->fr_L_thigh_X_fr_L_hip).transpose() * L_thigh_f;

    iit::rbd::transformInertia<Scalar>(L_hip_Ic, (xm->fr_L_hip_X_fr_trunk).transpose(), Ic_spare);
    trunk_Ic += Ic_spare;
    trunk_f = trunk_f + (xm->fr_L_hip_X_fr_trunk).transpose() * L_hip_f;


    // The base acceleration due to the force due to the movement of the links
    trunk_a = -trunk_Ic.inverse() * trunk_f;

    L_hip_a = xm->fr_L_hip_X_fr_trunk * trunk_a;
    Scalar L_hip_b = L_hip_Ic.row(iit::rbd::AZ) * L_hip_a;
    jForces(L_HAA) = (L_hip_b + L_hip_f(iit::rbd::AZ));

    L_thigh_a = xm->fr_L_thigh_X_fr_L_hip * L_hip_a;
    Scalar L_thigh_b = L_thigh_Ic.row(iit::rbd::AZ) * L_thigh_a;
    jForces(L_HFE) = (L_thigh_b + L_thigh_f(iit::rbd::AZ));

    L_shin_a = xm->fr_L_shin_X_fr_L_thigh * L_thigh_a;
    Scalar L_shin_b = L_shin_Ic.row(iit::rbd::AZ) * L_shin_a;
    jForces(L_KFE) = (L_shin_b + L_shin_f(iit::rbd::AZ));

    R_hip_a = xm->fr_R_hip_X_fr_trunk * trunk_a;
    Scalar R_hip_b = R_hip_Ic.row(iit::rbd::AZ) * R_hip_a;
    jForces(R_HAA) = (R_hip_b + R_hip_f(iit::rbd::AZ));

    R_thigh_a = xm->fr_R_thigh_X_fr_R_hip * R_hip_a;
    Scalar R_thigh_b = R_thigh_Ic.row(iit::rbd::AZ) * R_thigh_a;
    jForces(R_HFE) = (R_thigh_b + R_thigh_f(iit::rbd::AZ));

    R_shin_a = xm->fr_R_shin_X_fr_R_thigh * R_thigh_a;
    Scalar R_shin_b = R_shin_Ic.row(iit::rbd::AZ) * R_shin_a;
    jForces(R_KFE) = (R_shin_b + R_shin_f(iit::rbd::AZ));


    trunk_a += g;
}


void Biped::rcg::InverseDynamics::G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g) {
    const Acceleration& trunk_a = -g;

    // Link 'L_hip'
    L_hip_a = (xm->fr_L_hip_X_fr_trunk) * trunk_a;
    L_hip_f = L_hip_I * L_hip_a;
    // Link 'L_thigh'
    L_thigh_a = (xm->fr_L_thigh_X_fr_L_hip) * L_hip_a;
    L_thigh_f = L_thigh_I * L_thigh_a;
    // Link 'L_shin'
    L_shin_a = (xm->fr_L_shin_X_fr_L_thigh) * L_thigh_a;
    L_shin_f = L_shin_I * L_shin_a;
    // Link 'R_hip'
    R_hip_a = (xm->fr_R_hip_X_fr_trunk) * trunk_a;
    R_hip_f = R_hip_I * R_hip_a;
    // Link 'R_thigh'
    R_thigh_a = (xm->fr_R_thigh_X_fr_R_hip) * R_hip_a;
    R_thigh_f = R_thigh_I * R_thigh_a;
    // Link 'R_shin'
    R_shin_a = (xm->fr_R_shin_X_fr_R_thigh) * R_thigh_a;
    R_shin_f = R_shin_I * R_shin_a;

    trunk_f = trunk_I * trunk_a;

    secondPass_fullyActuated(jForces);

    baseWrench = trunk_f;
}

void Biped::rcg::InverseDynamics::C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& trunk_v, const JointState& qd) {
    // Link 'L_hip'
    L_hip_v = ((xm->fr_L_hip_X_fr_trunk) * trunk_v);
    L_hip_v(iit::rbd::AZ) += qd(L_HAA);
    motionCrossProductMx<Scalar>(L_hip_v, vcross);
    L_hip_a = (vcross.col(iit::rbd::AZ) * qd(L_HAA));
    L_hip_f = L_hip_I * L_hip_a + vxIv(L_hip_v, L_hip_I);

    // Link 'L_thigh'
    L_thigh_v = ((xm->fr_L_thigh_X_fr_L_hip) * L_hip_v);
    L_thigh_v(iit::rbd::AZ) += qd(L_HFE);
    motionCrossProductMx<Scalar>(L_thigh_v, vcross);
    L_thigh_a = (xm->fr_L_thigh_X_fr_L_hip) * L_hip_a + vcross.col(iit::rbd::AZ) * qd(L_HFE);
    L_thigh_f = L_thigh_I * L_thigh_a + vxIv(L_thigh_v, L_thigh_I);

    // Link 'L_shin'
    L_shin_v = ((xm->fr_L_shin_X_fr_L_thigh) * L_thigh_v);
    L_shin_v(iit::rbd::AZ) += qd(L_KFE);
    motionCrossProductMx<Scalar>(L_shin_v, vcross);
    L_shin_a = (xm->fr_L_shin_X_fr_L_thigh) * L_thigh_a + vcross.col(iit::rbd::AZ) * qd(L_KFE);
    L_shin_f = L_shin_I * L_shin_a + vxIv(L_shin_v, L_shin_I);

    // Link 'R_hip'
    R_hip_v = ((xm->fr_R_hip_X_fr_trunk) * trunk_v);
    R_hip_v(iit::rbd::AZ) += qd(R_HAA);
    motionCrossProductMx<Scalar>(R_hip_v, vcross);
    R_hip_a = (vcross.col(iit::rbd::AZ) * qd(R_HAA));
    R_hip_f = R_hip_I * R_hip_a + vxIv(R_hip_v, R_hip_I);

    // Link 'R_thigh'
    R_thigh_v = ((xm->fr_R_thigh_X_fr_R_hip) * R_hip_v);
    R_thigh_v(iit::rbd::AZ) += qd(R_HFE);
    motionCrossProductMx<Scalar>(R_thigh_v, vcross);
    R_thigh_a = (xm->fr_R_thigh_X_fr_R_hip) * R_hip_a + vcross.col(iit::rbd::AZ) * qd(R_HFE);
    R_thigh_f = R_thigh_I * R_thigh_a + vxIv(R_thigh_v, R_thigh_I);

    // Link 'R_shin'
    R_shin_v = ((xm->fr_R_shin_X_fr_R_thigh) * R_thigh_v);
    R_shin_v(iit::rbd::AZ) += qd(R_KFE);
    motionCrossProductMx<Scalar>(R_shin_v, vcross);
    R_shin_a = (xm->fr_R_shin_X_fr_R_thigh) * R_thigh_a + vcross.col(iit::rbd::AZ) * qd(R_KFE);
    R_shin_f = R_shin_I * R_shin_a + vxIv(R_shin_v, R_shin_I);


    trunk_f = vxIv(trunk_v, trunk_I);

    secondPass_fullyActuated(jForces);

    baseWrench = trunk_f;
}

void Biped::rcg::InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& trunk_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext) {
    Acceleration trunk_a = baseAccel - g;

    // First pass, link 'L_hip'
    L_hip_v = ((xm->fr_L_hip_X_fr_trunk) * trunk_v);
    L_hip_v(iit::rbd::AZ) += qd(L_HAA);

    motionCrossProductMx<Scalar>(L_hip_v, vcross);

    L_hip_a = (xm->fr_L_hip_X_fr_trunk) * trunk_a + vcross.col(iit::rbd::AZ) * qd(L_HAA);
    L_hip_a(iit::rbd::AZ) += qdd(L_HAA);

    L_hip_f = L_hip_I * L_hip_a + vxIv(L_hip_v, L_hip_I) - fext[L_HIP];

    // First pass, link 'L_thigh'
    L_thigh_v = ((xm->fr_L_thigh_X_fr_L_hip) * L_hip_v);
    L_thigh_v(iit::rbd::AZ) += qd(L_HFE);

    motionCrossProductMx<Scalar>(L_thigh_v, vcross);

    L_thigh_a = (xm->fr_L_thigh_X_fr_L_hip) * L_hip_a + vcross.col(iit::rbd::AZ) * qd(L_HFE);
    L_thigh_a(iit::rbd::AZ) += qdd(L_HFE);

    L_thigh_f = L_thigh_I * L_thigh_a + vxIv(L_thigh_v, L_thigh_I) - fext[L_THIGH];

    // First pass, link 'L_shin'
    L_shin_v = ((xm->fr_L_shin_X_fr_L_thigh) * L_thigh_v);
    L_shin_v(iit::rbd::AZ) += qd(L_KFE);

    motionCrossProductMx<Scalar>(L_shin_v, vcross);

    L_shin_a = (xm->fr_L_shin_X_fr_L_thigh) * L_thigh_a + vcross.col(iit::rbd::AZ) * qd(L_KFE);
    L_shin_a(iit::rbd::AZ) += qdd(L_KFE);

    L_shin_f = L_shin_I * L_shin_a + vxIv(L_shin_v, L_shin_I) - fext[L_SHIN];

    // First pass, link 'R_hip'
    R_hip_v = ((xm->fr_R_hip_X_fr_trunk) * trunk_v);
    R_hip_v(iit::rbd::AZ) += qd(R_HAA);

    motionCrossProductMx<Scalar>(R_hip_v, vcross);

    R_hip_a = (xm->fr_R_hip_X_fr_trunk) * trunk_a + vcross.col(iit::rbd::AZ) * qd(R_HAA);
    R_hip_a(iit::rbd::AZ) += qdd(R_HAA);

    R_hip_f = R_hip_I * R_hip_a + vxIv(R_hip_v, R_hip_I) - fext[R_HIP];

    // First pass, link 'R_thigh'
    R_thigh_v = ((xm->fr_R_thigh_X_fr_R_hip) * R_hip_v);
    R_thigh_v(iit::rbd::AZ) += qd(R_HFE);

    motionCrossProductMx<Scalar>(R_thigh_v, vcross);

    R_thigh_a = (xm->fr_R_thigh_X_fr_R_hip) * R_hip_a + vcross.col(iit::rbd::AZ) * qd(R_HFE);
    R_thigh_a(iit::rbd::AZ) += qdd(R_HFE);

    R_thigh_f = R_thigh_I * R_thigh_a + vxIv(R_thigh_v, R_thigh_I) - fext[R_THIGH];

    // First pass, link 'R_shin'
    R_shin_v = ((xm->fr_R_shin_X_fr_R_thigh) * R_thigh_v);
    R_shin_v(iit::rbd::AZ) += qd(R_KFE);

    motionCrossProductMx<Scalar>(R_shin_v, vcross);

    R_shin_a = (xm->fr_R_shin_X_fr_R_thigh) * R_thigh_a + vcross.col(iit::rbd::AZ) * qd(R_KFE);
    R_shin_a(iit::rbd::AZ) += qdd(R_KFE);

    R_shin_f = R_shin_I * R_shin_a + vxIv(R_shin_v, R_shin_I) - fext[R_SHIN];


    // The base
    trunk_f = trunk_I * trunk_a + vxIv(trunk_v, trunk_I) - fext[TRUNK];

    secondPass_fullyActuated(jForces);

    baseWrench = trunk_f;
}


void Biped::rcg::InverseDynamics::secondPass_fullyActuated(JointState& jForces) {
    // Link 'R_shin'
    jForces(R_KFE) = R_shin_f(iit::rbd::AZ);
    R_thigh_f += xm->fr_R_shin_X_fr_R_thigh.transpose() * R_shin_f;
    // Link 'R_thigh'
    jForces(R_HFE) = R_thigh_f(iit::rbd::AZ);
    R_hip_f += xm->fr_R_thigh_X_fr_R_hip.transpose() * R_thigh_f;
    // Link 'R_hip'
    jForces(R_HAA) = R_hip_f(iit::rbd::AZ);
    trunk_f += xm->fr_R_hip_X_fr_trunk.transpose() * R_hip_f;
    // Link 'L_shin'
    jForces(L_KFE) = L_shin_f(iit::rbd::AZ);
    L_thigh_f += xm->fr_L_shin_X_fr_L_thigh.transpose() * L_shin_f;
    // Link 'L_thigh'
    jForces(L_HFE) = L_thigh_f(iit::rbd::AZ);
    L_hip_f += xm->fr_L_thigh_X_fr_L_hip.transpose() * L_thigh_f;
    // Link 'L_hip'
    jForces(L_HAA) = L_hip_f(iit::rbd::AZ);
    trunk_f += xm->fr_L_hip_X_fr_trunk.transpose() * L_hip_f;
}
