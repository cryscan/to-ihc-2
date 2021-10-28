#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace Biped::rcg;

Vector3 Biped::rcg::getWholeBodyCOM(
        const InertiaProperties& inertiaProps,
        const HomogeneousTransforms& ht) {
    Vector3 tmpSum(Vector3::Zero());

    tmpSum += inertiaProps.getCOM_trunk() * inertiaProps.getMass_trunk();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_L_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_R_HAA_chain;


    base_X_L_HAA_chain = tmpX * ht.fr_trunk_X_fr_L_hip;
    tmpSum += inertiaProps.getMass_L_hip() *
              (iit::rbd::Utils::transform(base_X_L_HAA_chain, inertiaProps.getCOM_L_hip()));

    base_X_L_HAA_chain = base_X_L_HAA_chain * ht.fr_L_hip_X_fr_L_thigh;
    tmpSum += inertiaProps.getMass_L_thigh() *
              (iit::rbd::Utils::transform(base_X_L_HAA_chain, inertiaProps.getCOM_L_thigh()));

    base_X_L_HAA_chain = base_X_L_HAA_chain * ht.fr_L_thigh_X_fr_L_shin;
    tmpSum += inertiaProps.getMass_L_shin() *
              (iit::rbd::Utils::transform(base_X_L_HAA_chain, inertiaProps.getCOM_L_shin()));

    base_X_R_HAA_chain = tmpX * ht.fr_trunk_X_fr_R_hip;
    tmpSum += inertiaProps.getMass_R_hip() *
              (iit::rbd::Utils::transform(base_X_R_HAA_chain, inertiaProps.getCOM_R_hip()));

    base_X_R_HAA_chain = base_X_R_HAA_chain * ht.fr_R_hip_X_fr_R_thigh;
    tmpSum += inertiaProps.getMass_R_thigh() *
              (iit::rbd::Utils::transform(base_X_R_HAA_chain, inertiaProps.getCOM_R_thigh()));

    base_X_R_HAA_chain = base_X_R_HAA_chain * ht.fr_R_thigh_X_fr_R_shin;
    tmpSum += inertiaProps.getMass_R_shin() *
              (iit::rbd::Utils::transform(base_X_R_HAA_chain, inertiaProps.getCOM_R_shin()));


    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 Biped::rcg::getWholeBodyCOM(
        const InertiaProperties& inertiaProps,
        const JointState& q,
        HomogeneousTransforms& ht) {
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_trunk_X_fr_L_hip(q);
    ht.fr_trunk_X_fr_R_hip(q);
    ht.fr_L_hip_X_fr_L_thigh(q);
    ht.fr_L_thigh_X_fr_L_shin(q);
    ht.fr_R_hip_X_fr_R_thigh(q);
    ht.fr_R_thigh_X_fr_R_shin(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
