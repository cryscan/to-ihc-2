#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace Hopper::rcg;

Vector3 Hopper::rcg::getWholeBodyCOM(
        const InertiaProperties& inertiaProps,
        const HomogeneousTransforms& ht) {
    Vector3 tmpSum(Vector3::Zero());


    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_u0_X_fr_u1;
    tmpSum += inertiaProps.getMass_u1() *
              (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_u1()));

    tmpX = tmpX * ht.fr_u1_X_fr_u2;
    tmpSum += inertiaProps.getMass_u2() *
              (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_u2()));

    tmpX = tmpX * ht.fr_u2_X_fr_body;
    tmpSum += inertiaProps.getMass_body() *
              (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_body()));

    tmpX = tmpX * ht.fr_body_X_fr_leg;
    tmpSum += inertiaProps.getMass_leg() *
              (iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_leg()));


    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 Hopper::rcg::getWholeBodyCOM(
        const InertiaProperties& inertiaProps,
        const JointState& q,
        HomogeneousTransforms& ht) {
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_u0_X_fr_u1(q);
    ht.fr_u1_X_fr_u2(q);
    ht.fr_u2_X_fr_body(q);
    ht.fr_body_X_fr_leg(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
