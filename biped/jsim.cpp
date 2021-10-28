#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
Biped::rcg::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
        linkInertias(inertiaProperties),
        frcTransf(&forceTransforms),
        L_shin_Ic(linkInertias.getTensor_L_shin()),
        R_shin_Ic(linkInertias.getTensor_R_shin()) {
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
#define Fcol(j) (block<6,1>(0,(j)+6))
#define F(i, j) DATA((i),(j)+6)
const Biped::rcg::JSIM& Biped::rcg::JSIM::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf->fr_R_thigh_X_fr_R_shin(state);
    frcTransf->fr_R_hip_X_fr_R_thigh(state);
    frcTransf->fr_trunk_X_fr_R_hip(state);
    frcTransf->fr_L_thigh_X_fr_L_shin(state);
    frcTransf->fr_L_hip_X_fr_L_thigh(state);
    frcTransf->fr_trunk_X_fr_L_hip(state);

    // Initializes the composite inertia tensors
    trunk_Ic = linkInertias.getTensor_trunk();
    L_hip_Ic = linkInertias.getTensor_L_hip();
    L_thigh_Ic = linkInertias.getTensor_L_thigh();
    R_hip_Ic = linkInertias.getTensor_R_hip();
    R_thigh_Ic = linkInertias.getTensor_R_thigh();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link R_shin:
    iit::rbd::transformInertia<Scalar>(R_shin_Ic, frcTransf->fr_R_thigh_X_fr_R_shin, Ic_spare);
    R_thigh_Ic += Ic_spare;

    Fcol(R_KFE) = R_shin_Ic.col(AZ);
    DATA(R_KFE + 6, R_KFE + 6) = Fcol(R_KFE)(AZ);

    Fcol(R_KFE) = frcTransf->fr_R_thigh_X_fr_R_shin * Fcol(R_KFE);
    DATA(R_KFE + 6, R_HFE + 6) = F(AZ, R_KFE);
    DATA(R_HFE + 6, R_KFE + 6) = DATA(R_KFE + 6, R_HFE + 6);
    Fcol(R_KFE) = frcTransf->fr_R_hip_X_fr_R_thigh * Fcol(R_KFE);
    DATA(R_KFE + 6, R_HAA + 6) = F(AZ, R_KFE);
    DATA(R_HAA + 6, R_KFE + 6) = DATA(R_KFE + 6, R_HAA + 6);
    Fcol(R_KFE) = frcTransf->fr_trunk_X_fr_R_hip * Fcol(R_KFE);

    // Link R_thigh:
    iit::rbd::transformInertia<Scalar>(R_thigh_Ic, frcTransf->fr_R_hip_X_fr_R_thigh, Ic_spare);
    R_hip_Ic += Ic_spare;

    Fcol(R_HFE) = R_thigh_Ic.col(AZ);
    DATA(R_HFE + 6, R_HFE + 6) = Fcol(R_HFE)(AZ);

    Fcol(R_HFE) = frcTransf->fr_R_hip_X_fr_R_thigh * Fcol(R_HFE);
    DATA(R_HFE + 6, R_HAA + 6) = F(AZ, R_HFE);
    DATA(R_HAA + 6, R_HFE + 6) = DATA(R_HFE + 6, R_HAA + 6);
    Fcol(R_HFE) = frcTransf->fr_trunk_X_fr_R_hip * Fcol(R_HFE);

    // Link R_hip:
    iit::rbd::transformInertia<Scalar>(R_hip_Ic, frcTransf->fr_trunk_X_fr_R_hip, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(R_HAA) = R_hip_Ic.col(AZ);
    DATA(R_HAA + 6, R_HAA + 6) = Fcol(R_HAA)(AZ);

    Fcol(R_HAA) = frcTransf->fr_trunk_X_fr_R_hip * Fcol(R_HAA);

    // Link L_shin:
    iit::rbd::transformInertia<Scalar>(L_shin_Ic, frcTransf->fr_L_thigh_X_fr_L_shin, Ic_spare);
    L_thigh_Ic += Ic_spare;

    Fcol(L_KFE) = L_shin_Ic.col(AZ);
    DATA(L_KFE + 6, L_KFE + 6) = Fcol(L_KFE)(AZ);

    Fcol(L_KFE) = frcTransf->fr_L_thigh_X_fr_L_shin * Fcol(L_KFE);
    DATA(L_KFE + 6, L_HFE + 6) = F(AZ, L_KFE);
    DATA(L_HFE + 6, L_KFE + 6) = DATA(L_KFE + 6, L_HFE + 6);
    Fcol(L_KFE) = frcTransf->fr_L_hip_X_fr_L_thigh * Fcol(L_KFE);
    DATA(L_KFE + 6, L_HAA + 6) = F(AZ, L_KFE);
    DATA(L_HAA + 6, L_KFE + 6) = DATA(L_KFE + 6, L_HAA + 6);
    Fcol(L_KFE) = frcTransf->fr_trunk_X_fr_L_hip * Fcol(L_KFE);

    // Link L_thigh:
    iit::rbd::transformInertia<Scalar>(L_thigh_Ic, frcTransf->fr_L_hip_X_fr_L_thigh, Ic_spare);
    L_hip_Ic += Ic_spare;

    Fcol(L_HFE) = L_thigh_Ic.col(AZ);
    DATA(L_HFE + 6, L_HFE + 6) = Fcol(L_HFE)(AZ);

    Fcol(L_HFE) = frcTransf->fr_L_hip_X_fr_L_thigh * Fcol(L_HFE);
    DATA(L_HFE + 6, L_HAA + 6) = F(AZ, L_HFE);
    DATA(L_HAA + 6, L_HFE + 6) = DATA(L_HFE + 6, L_HAA + 6);
    Fcol(L_HFE) = frcTransf->fr_trunk_X_fr_L_hip * Fcol(L_HFE);

    // Link L_hip:
    iit::rbd::transformInertia<Scalar>(L_hip_Ic, frcTransf->fr_trunk_X_fr_L_hip, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(L_HAA) = L_hip_Ic.col(AZ);
    DATA(L_HAA + 6, L_HAA + 6) = Fcol(L_HAA)(AZ);

    Fcol(L_HAA) = frcTransf->fr_trunk_X_fr_L_hip * Fcol(L_HAA);

    // Copies the upper-right block into the lower-left block, after transposing
    block<6, 6>(6, 0) = (block<6, 6>(0, 6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    block<6, 6>(0, 0) = trunk_Ic;
    return *this;
}

#undef DATA
#undef F

void Biped::rcg::JSIM::computeL() {
    L = this->triangularView<Eigen::Lower>();
    // Joint R_KFE, index 5 :
    L(5, 5) = ScalarTraits::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);

    // Joint R_HFE, index 4 :
    L(4, 4) = ScalarTraits::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);

    // Joint R_HAA, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));

    // Joint L_KFE, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);

    // Joint L_HFE, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);

    // Joint L_HAA, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));

}

void Biped::rcg::JSIM::computeInverse() {
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
    inverse(3, 3) = +(Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) = +(Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) = +(Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) = +(Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) = +(Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) = +(Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
}

void Biped::rcg::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(1, 0) = -Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = -Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = -Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(4, 3) = -Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = -Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = -Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
}
