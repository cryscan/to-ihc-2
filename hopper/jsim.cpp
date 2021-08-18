#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
Hopper::rcg::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
        linkInertias(inertiaProperties),
        frcTransf(&forceTransforms),
        leg_Ic(linkInertias.getTensor_leg()) {
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
const Hopper::rcg::JSIM& Hopper::rcg::JSIM::update(const JointState& state) {
    Force F;

    // Precomputes only once the coordinate transforms:
    frcTransf->fr_body_X_fr_leg(state);
    frcTransf->fr_u2_X_fr_body(state);
    frcTransf->fr_u1_X_fr_u2(state);

    // Initializes the composite inertia tensors
    u1_Ic = linkInertias.getTensor_u1();
    u2_Ic = linkInertias.getTensor_u2();
    body_Ic = linkInertias.getTensor_body();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link leg:
    iit::rbd::transformInertia<Scalar>(leg_Ic, frcTransf->fr_body_X_fr_leg, Ic_spare);
    body_Ic += Ic_spare;

    F = leg_Ic.col(AZ);
    DATA(KFE, KFE) = F(AZ);

    F = frcTransf->fr_body_X_fr_leg * F;
    DATA(KFE, HFE) = F(AZ);
    DATA(HFE, KFE) = DATA(KFE, HFE);
    F = frcTransf->fr_u2_X_fr_body * F;
    DATA(KFE, BX) = F(LZ);
    DATA(BX, KFE) = DATA(KFE, BX);
    F = frcTransf->fr_u1_X_fr_u2 * F;
    DATA(KFE, BH) = F(LZ);
    DATA(BH, KFE) = DATA(KFE, BH);

    // Link body:
    iit::rbd::transformInertia<Scalar>(body_Ic, frcTransf->fr_u2_X_fr_body, Ic_spare);
    u2_Ic += Ic_spare;

    F = body_Ic.col(AZ);
    DATA(HFE, HFE) = F(AZ);

    F = frcTransf->fr_u2_X_fr_body * F;
    DATA(HFE, BX) = F(LZ);
    DATA(BX, HFE) = DATA(HFE, BX);
    F = frcTransf->fr_u1_X_fr_u2 * F;
    DATA(HFE, BH) = F(LZ);
    DATA(BH, HFE) = DATA(HFE, BH);

    // Link u2:
    iit::rbd::transformInertia<Scalar>(u2_Ic, frcTransf->fr_u1_X_fr_u2, Ic_spare);
    u1_Ic += Ic_spare;

    F = u2_Ic.col(LZ);
    DATA(BX, BX) = F(LZ);

    F = frcTransf->fr_u1_X_fr_u2 * F;
    DATA(BX, BH) = F(LZ);
    DATA(BH, BX) = DATA(BX, BH);

    // Link u1:

    F = u1_Ic.col(LZ);
    DATA(BH, BH) = F(LZ);


    return *this;
}

#undef DATA
#undef F

void Hopper::rcg::JSIM::computeL() {
    L = this->triangularView<Eigen::Lower>();
    // Joint KFE, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));
    L(3, 2) = L(3, 2) / L(3, 3);
    L(3, 1) = L(3, 1) / L(3, 3);
    L(3, 0) = L(3, 0) / L(3, 3);
    L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
    L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
    L(2, 0) = L(2, 0) - L(3, 2) * L(3, 0);
    L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);
    L(1, 0) = L(1, 0) - L(3, 1) * L(3, 0);
    L(0, 0) = L(0, 0) - L(3, 0) * L(3, 0);

    // Joint HFE, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);

    // Joint BX, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);

    // Joint BH, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));

}

void Hopper::rcg::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) = +(Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) = +(Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) = +(Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) = +(Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) = +(Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) = +(Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) = +(Linv(3, 0) * Linv(3, 0)) + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) +
                    (Linv(3, 3) * Linv(3, 3));
    inverse(3, 2) = +(Linv(3, 0) * Linv(2, 0)) + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) = +(Linv(3, 0) * Linv(1, 0)) + (Linv(3, 1) * Linv(1, 1));
    inverse(1, 3) = inverse(3, 1);
    inverse(3, 0) = +(Linv(3, 0) * Linv(0, 0));
    inverse(0, 3) = inverse(3, 0);
}

void Hopper::rcg::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(1, 0) = -Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = -Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = -Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(3, 2) = -Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = -Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
    Linv(3, 0) = -Linv(0, 0) * ((Linv(3, 1) * L(1, 0)) + (Linv(3, 2) * L(2, 0)) + (Linv(3, 3) * L(3, 0)) + 0);
}
