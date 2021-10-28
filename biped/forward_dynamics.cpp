#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const Biped::rcg::ForwardDynamics::ExtForces
        Biped::rcg::ForwardDynamics::zeroExtForces(Force::Zero());

Biped::rcg::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
        inertiaProps(&inertia),
        motionTransforms(&transforms) {
    L_hip_v.setZero();
    L_hip_c.setZero();
    L_thigh_v.setZero();
    L_thigh_c.setZero();
    L_shin_v.setZero();
    L_shin_c.setZero();
    R_hip_v.setZero();
    R_hip_c.setZero();
    R_thigh_v.setZero();
    R_thigh_c.setZero();
    R_shin_v.setZero();
    R_shin_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void Biped::rcg::ForwardDynamics::fd(
        JointState& qdd,
        Acceleration& trunk_a,
        const Velocity& trunk_v,
        const Acceleration& g,
        const JointState& qd,
        const JointState& tau,
        const ExtForces& fext/* = zeroExtForces */) {

    trunk_AI = inertiaProps->getTensor_trunk();
    trunk_p = -fext[TRUNK];
    L_hip_AI = inertiaProps->getTensor_L_hip();
    L_hip_p = -fext[L_HIP];
    L_thigh_AI = inertiaProps->getTensor_L_thigh();
    L_thigh_p = -fext[L_THIGH];
    L_shin_AI = inertiaProps->getTensor_L_shin();
    L_shin_p = -fext[L_SHIN];
    R_hip_AI = inertiaProps->getTensor_R_hip();
    R_hip_p = -fext[R_HIP];
    R_thigh_AI = inertiaProps->getTensor_R_thigh();
    R_thigh_p = -fext[R_THIGH];
    R_shin_AI = inertiaProps->getTensor_R_shin();
    R_shin_p = -fext[R_SHIN];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.

    // + Link L_hip
    //  - The spatial velocity:
    L_hip_v = (motionTransforms->fr_L_hip_X_fr_trunk) * trunk_v;
    L_hip_v(AZ) += qd(L_HAA);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(L_hip_v, vcross);
    L_hip_c = vcross.col(AZ) * qd(L_HAA);

    //  - The bias force term:
    L_hip_p += vxIv(L_hip_v, L_hip_AI);

    // + Link L_thigh
    //  - The spatial velocity:
    L_thigh_v = (motionTransforms->fr_L_thigh_X_fr_L_hip) * L_hip_v;
    L_thigh_v(AZ) += qd(L_HFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(L_thigh_v, vcross);
    L_thigh_c = vcross.col(AZ) * qd(L_HFE);

    //  - The bias force term:
    L_thigh_p += vxIv(L_thigh_v, L_thigh_AI);

    // + Link L_shin
    //  - The spatial velocity:
    L_shin_v = (motionTransforms->fr_L_shin_X_fr_L_thigh) * L_thigh_v;
    L_shin_v(AZ) += qd(L_KFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(L_shin_v, vcross);
    L_shin_c = vcross.col(AZ) * qd(L_KFE);

    //  - The bias force term:
    L_shin_p += vxIv(L_shin_v, L_shin_AI);

    // + Link R_hip
    //  - The spatial velocity:
    R_hip_v = (motionTransforms->fr_R_hip_X_fr_trunk) * trunk_v;
    R_hip_v(AZ) += qd(R_HAA);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(R_hip_v, vcross);
    R_hip_c = vcross.col(AZ) * qd(R_HAA);

    //  - The bias force term:
    R_hip_p += vxIv(R_hip_v, R_hip_AI);

    // + Link R_thigh
    //  - The spatial velocity:
    R_thigh_v = (motionTransforms->fr_R_thigh_X_fr_R_hip) * R_hip_v;
    R_thigh_v(AZ) += qd(R_HFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(R_thigh_v, vcross);
    R_thigh_c = vcross.col(AZ) * qd(R_HFE);

    //  - The bias force term:
    R_thigh_p += vxIv(R_thigh_v, R_thigh_AI);

    // + Link R_shin
    //  - The spatial velocity:
    R_shin_v = (motionTransforms->fr_R_shin_X_fr_R_thigh) * R_thigh_v;
    R_shin_v(AZ) += qd(R_KFE);

    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(R_shin_v, vcross);
    R_shin_c = vcross.col(AZ) * qd(R_KFE);

    //  - The bias force term:
    R_shin_p += vxIv(R_shin_v, R_shin_AI);

    // + The floating base body
    trunk_p += vxIv(trunk_v, trunk_AI);

    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;

    // + Link R_shin
    R_shin_u = tau(R_KFE) - R_shin_p(AZ);
    R_shin_U = R_shin_AI.col(AZ);
    R_shin_D = R_shin_U(AZ);

    compute_Ia_revolute(R_shin_AI, R_shin_U, R_shin_D,
                        Ia_r);  // same as: Ia_r = R_shin_AI - R_shin_U/R_shin_D * R_shin_U.transpose();
    pa = R_shin_p + Ia_r * R_shin_c + R_shin_U * R_shin_u / R_shin_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms->fr_R_shin_X_fr_R_thigh, IaB);
    R_thigh_AI += IaB;
    R_thigh_p += (motionTransforms->fr_R_shin_X_fr_R_thigh).transpose() * pa;

    // + Link R_thigh
    R_thigh_u = tau(R_HFE) - R_thigh_p(AZ);
    R_thigh_U = R_thigh_AI.col(AZ);
    R_thigh_D = R_thigh_U(AZ);

    compute_Ia_revolute(R_thigh_AI, R_thigh_U, R_thigh_D,
                        Ia_r);  // same as: Ia_r = R_thigh_AI - R_thigh_U/R_thigh_D * R_thigh_U.transpose();
    pa = R_thigh_p + Ia_r * R_thigh_c + R_thigh_U * R_thigh_u / R_thigh_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms->fr_R_thigh_X_fr_R_hip, IaB);
    R_hip_AI += IaB;
    R_hip_p += (motionTransforms->fr_R_thigh_X_fr_R_hip).transpose() * pa;

    // + Link R_hip
    R_hip_u = tau(R_HAA) - R_hip_p(AZ);
    R_hip_U = R_hip_AI.col(AZ);
    R_hip_D = R_hip_U(AZ);

    compute_Ia_revolute(R_hip_AI, R_hip_U, R_hip_D,
                        Ia_r);  // same as: Ia_r = R_hip_AI - R_hip_U/R_hip_D * R_hip_U.transpose();
    pa = R_hip_p + Ia_r * R_hip_c + R_hip_U * R_hip_u / R_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms->fr_R_hip_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms->fr_R_hip_X_fr_trunk).transpose() * pa;

    // + Link L_shin
    L_shin_u = tau(L_KFE) - L_shin_p(AZ);
    L_shin_U = L_shin_AI.col(AZ);
    L_shin_D = L_shin_U(AZ);

    compute_Ia_revolute(L_shin_AI, L_shin_U, L_shin_D,
                        Ia_r);  // same as: Ia_r = L_shin_AI - L_shin_U/L_shin_D * L_shin_U.transpose();
    pa = L_shin_p + Ia_r * L_shin_c + L_shin_U * L_shin_u / L_shin_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms->fr_L_shin_X_fr_L_thigh, IaB);
    L_thigh_AI += IaB;
    L_thigh_p += (motionTransforms->fr_L_shin_X_fr_L_thigh).transpose() * pa;

    // + Link L_thigh
    L_thigh_u = tau(L_HFE) - L_thigh_p(AZ);
    L_thigh_U = L_thigh_AI.col(AZ);
    L_thigh_D = L_thigh_U(AZ);

    compute_Ia_revolute(L_thigh_AI, L_thigh_U, L_thigh_D,
                        Ia_r);  // same as: Ia_r = L_thigh_AI - L_thigh_U/L_thigh_D * L_thigh_U.transpose();
    pa = L_thigh_p + Ia_r * L_thigh_c + L_thigh_U * L_thigh_u / L_thigh_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms->fr_L_thigh_X_fr_L_hip, IaB);
    L_hip_AI += IaB;
    L_hip_p += (motionTransforms->fr_L_thigh_X_fr_L_hip).transpose() * pa;

    // + Link L_hip
    L_hip_u = tau(L_HAA) - L_hip_p(AZ);
    L_hip_U = L_hip_AI.col(AZ);
    L_hip_D = L_hip_U(AZ);

    compute_Ia_revolute(L_hip_AI, L_hip_U, L_hip_D,
                        Ia_r);  // same as: Ia_r = L_hip_AI - L_hip_U/L_hip_D * L_hip_U.transpose();
    pa = L_hip_p + Ia_r * L_hip_c + L_hip_U * L_hip_u / L_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms->fr_L_hip_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms->fr_L_hip_X_fr_trunk).transpose() * pa;

    // + The acceleration of the floating base trunk, without gravity
    trunk_a = -trunk_AI.llt().solve(trunk_p);  // trunk_a = - IA^-1 * trunk_p

    // ---------------------- THIRD PASS ---------------------- //
    L_hip_a = (motionTransforms->fr_L_hip_X_fr_trunk) * trunk_a + L_hip_c;
    qdd(L_HAA) = (L_hip_u - L_hip_U.dot(L_hip_a)) / L_hip_D;
    L_hip_a(AZ) += qdd(L_HAA);

    L_thigh_a = (motionTransforms->fr_L_thigh_X_fr_L_hip) * L_hip_a + L_thigh_c;
    qdd(L_HFE) = (L_thigh_u - L_thigh_U.dot(L_thigh_a)) / L_thigh_D;
    L_thigh_a(AZ) += qdd(L_HFE);

    L_shin_a = (motionTransforms->fr_L_shin_X_fr_L_thigh) * L_thigh_a + L_shin_c;
    qdd(L_KFE) = (L_shin_u - L_shin_U.dot(L_shin_a)) / L_shin_D;
    L_shin_a(AZ) += qdd(L_KFE);

    R_hip_a = (motionTransforms->fr_R_hip_X_fr_trunk) * trunk_a + R_hip_c;
    qdd(R_HAA) = (R_hip_u - R_hip_U.dot(R_hip_a)) / R_hip_D;
    R_hip_a(AZ) += qdd(R_HAA);

    R_thigh_a = (motionTransforms->fr_R_thigh_X_fr_R_hip) * R_hip_a + R_thigh_c;
    qdd(R_HFE) = (R_thigh_u - R_thigh_U.dot(R_thigh_a)) / R_thigh_D;
    R_thigh_a(AZ) += qdd(R_HFE);

    R_shin_a = (motionTransforms->fr_R_shin_X_fr_R_thigh) * R_thigh_a + R_shin_c;
    qdd(R_KFE) = (R_shin_u - R_shin_U.dot(R_shin_a)) / R_shin_D;
    R_shin_a(AZ) += qdd(R_KFE);


    // + Add gravity to the acceleration of the floating base
    trunk_a += g;
}
