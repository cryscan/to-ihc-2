#include "transforms.h"

using namespace Biped::rcg;

// Constructors

MotionTransforms::MotionTransforms()
        : fr_trunk_X_L_foot(),
          fr_trunk_X_fr_L_HAA(),
          fr_trunk_X_fr_L_HFE(),
          fr_trunk_X_fr_L_KFE(),
          fr_trunk_X_R_foot(),
          fr_trunk_X_fr_R_HAA(),
          fr_trunk_X_fr_R_HFE(),
          fr_trunk_X_fr_R_KFE(),
          fr_L_hip_X_fr_trunk(),
          fr_trunk_X_fr_L_hip(),
          fr_L_thigh_X_fr_L_hip(),
          fr_L_hip_X_fr_L_thigh(),
          fr_L_shin_X_fr_L_thigh(),
          fr_L_thigh_X_fr_L_shin(),
          fr_R_hip_X_fr_trunk(),
          fr_trunk_X_fr_R_hip(),
          fr_R_thigh_X_fr_R_hip(),
          fr_R_hip_X_fr_R_thigh(),
          fr_R_shin_X_fr_R_thigh(),
          fr_R_thigh_X_fr_R_shin() {}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles) {
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
        : fr_trunk_X_L_foot(),
          fr_trunk_X_fr_L_HAA(),
          fr_trunk_X_fr_L_HFE(),
          fr_trunk_X_fr_L_KFE(),
          fr_trunk_X_R_foot(),
          fr_trunk_X_fr_R_HAA(),
          fr_trunk_X_fr_R_HFE(),
          fr_trunk_X_fr_R_KFE(),
          fr_L_hip_X_fr_trunk(),
          fr_trunk_X_fr_L_hip(),
          fr_L_thigh_X_fr_L_hip(),
          fr_L_hip_X_fr_L_thigh(),
          fr_L_shin_X_fr_L_thigh(),
          fr_L_thigh_X_fr_L_shin(),
          fr_R_hip_X_fr_trunk(),
          fr_trunk_X_fr_R_hip(),
          fr_R_thigh_X_fr_R_hip(),
          fr_R_hip_X_fr_R_thigh(),
          fr_R_shin_X_fr_R_thigh(),
          fr_R_thigh_X_fr_R_shin() {}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles) {
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
        : fr_trunk_X_L_foot(),
          fr_trunk_X_fr_L_HAA(),
          fr_trunk_X_fr_L_HFE(),
          fr_trunk_X_fr_L_KFE(),
          fr_trunk_X_R_foot(),
          fr_trunk_X_fr_R_HAA(),
          fr_trunk_X_fr_R_HFE(),
          fr_trunk_X_fr_R_KFE(),
          fr_L_hip_X_fr_trunk(),
          fr_trunk_X_fr_L_hip(),
          fr_L_thigh_X_fr_L_hip(),
          fr_L_hip_X_fr_L_thigh(),
          fr_L_shin_X_fr_L_thigh(),
          fr_L_thigh_X_fr_L_shin(),
          fr_R_hip_X_fr_trunk(),
          fr_trunk_X_fr_R_hip(),
          fr_R_thigh_X_fr_R_hip(),
          fr_R_hip_X_fr_R_thigh(),
          fr_R_shin_X_fr_R_thigh(),
          fr_R_thigh_X_fr_R_shin() {}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles) {
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_trunk_X_L_foot::Type_fr_trunk_X_L_foot() {
    (*this)(0, 1) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 4) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_L_foot& MotionTransforms::Type_fr_trunk_X_L_foot::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(0, 0) = (cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HFE * sin_q_L_KFE);
    (*this)(0, 2) = (cos_q_L_HFE * sin_q_L_KFE) + (sin_q_L_HFE * cos_q_L_KFE);
    (*this)(1, 0) = (-sin_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(1, 1) = cos_q_L_HAA;
    (*this)(1, 2) = (sin_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    (*this)(2, 0) = (-cos_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(2, 1) = -sin_q_L_HAA;
    (*this)(2, 2) = (cos_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    (*this)(3, 0) = (((tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA)) * cos_q_L_HFE * sin_q_L_KFE) +
                    (((tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA)) * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(3, 1) = (-tx_L_foot * sin_q_L_HFE * sin_q_L_KFE) + (tx_L_foot * cos_q_L_HFE * cos_q_L_KFE) +
                    (tx_L_KFE * cos_q_L_HFE) - (ty_L_HAA * sin_q_L_HAA) - (tz_L_HAA * cos_q_L_HAA) + tx_L_HFE;
    (*this)(3, 2) = (((tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA)) * sin_q_L_HFE * sin_q_L_KFE) +
                    (((ty_L_HAA * cos_q_L_HAA) - (tz_L_HAA * sin_q_L_HAA)) * cos_q_L_HFE * cos_q_L_KFE);
    (*this)(3, 3) = (cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HFE * sin_q_L_KFE);
    (*this)(3, 5) = (cos_q_L_HFE * sin_q_L_KFE) + (sin_q_L_HFE * cos_q_L_KFE);
    (*this)(4, 0) = (((tx_L_HFE * cos_q_L_HAA) - tz_L_HAA) * sin_q_L_HFE * sin_q_L_KFE) +
                    ((((tz_L_HAA - (tx_L_HFE * cos_q_L_HAA)) * cos_q_L_HFE) - (tx_L_KFE * cos_q_L_HAA)) * cos_q_L_KFE) -
                    (tx_L_foot * cos_q_L_HAA);
    (*this)(4, 1) = (-tx_L_foot * sin_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) -
                    (tx_L_foot * sin_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE) - (tx_L_KFE * sin_q_L_HAA * sin_q_L_HFE);
    (*this)(4, 2) = ((((tz_L_HAA - (tx_L_HFE * cos_q_L_HAA)) * cos_q_L_HFE) - (tx_L_KFE * cos_q_L_HAA)) * sin_q_L_KFE) +
                    ((tz_L_HAA - (tx_L_HFE * cos_q_L_HAA)) * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(4, 3) = (-sin_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(4, 4) = cos_q_L_HAA;
    (*this)(4, 5) = (sin_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    (*this)(5, 0) = ((ty_L_HAA - (tx_L_HFE * sin_q_L_HAA)) * sin_q_L_HFE * sin_q_L_KFE) +
                    (((((tx_L_HFE * sin_q_L_HAA) - ty_L_HAA) * cos_q_L_HFE) + (tx_L_KFE * sin_q_L_HAA)) * cos_q_L_KFE) +
                    (tx_L_foot * sin_q_L_HAA);
    (*this)(5, 1) = (-tx_L_foot * cos_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) -
                    (tx_L_foot * cos_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE) - (tx_L_KFE * cos_q_L_HAA * sin_q_L_HFE);
    (*this)(5, 2) = (((((tx_L_HFE * sin_q_L_HAA) - ty_L_HAA) * cos_q_L_HFE) + (tx_L_KFE * sin_q_L_HAA)) * sin_q_L_KFE) +
                    (((tx_L_HFE * sin_q_L_HAA) - ty_L_HAA) * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(5, 3) = (-cos_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(5, 4) = -sin_q_L_HAA;
    (*this)(5, 5) = (cos_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    return *this;
}
MotionTransforms::Type_fr_trunk_X_fr_L_HAA::Type_fr_trunk_X_fr_L_HAA() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = -1.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = -1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = -ty_L_HAA;    // Maxima DSL: -_k__ty_L_HAA
    (*this)(3, 1) = tz_L_HAA;    // Maxima DSL: _k__tz_L_HAA
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = -1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = -tz_L_HAA;    // Maxima DSL: -_k__tz_L_HAA
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = -1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = ty_L_HAA;    // Maxima DSL: _k__ty_L_HAA
    (*this)(5, 3) = -1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_fr_L_HAA& MotionTransforms::Type_fr_trunk_X_fr_L_HAA::update(const state_t& q) {
    return *this;
}
MotionTransforms::Type_fr_trunk_X_fr_L_HFE::Type_fr_trunk_X_fr_L_HFE() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = -1.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = -1.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 4) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_fr_L_HFE& MotionTransforms::Type_fr_trunk_X_fr_L_HFE::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    (*this)(1, 0) = -sin_q_L_HAA;
    (*this)(1, 2) = cos_q_L_HAA;
    (*this)(2, 0) = -cos_q_L_HAA;
    (*this)(2, 2) = -sin_q_L_HAA;
    (*this)(3, 0) = (tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA);
    (*this)(3, 2) = (-ty_L_HAA * sin_q_L_HAA) - (tz_L_HAA * cos_q_L_HAA) + tx_L_HFE;
    (*this)(4, 1) = (tx_L_HFE * cos_q_L_HAA) - tz_L_HAA;
    (*this)(4, 3) = -sin_q_L_HAA;
    (*this)(4, 5) = cos_q_L_HAA;
    (*this)(5, 1) = ty_L_HAA - (tx_L_HFE * sin_q_L_HAA);
    (*this)(5, 3) = -cos_q_L_HAA;
    (*this)(5, 5) = -sin_q_L_HAA;
    return *this;
}
MotionTransforms::Type_fr_trunk_X_fr_L_KFE::Type_fr_trunk_X_fr_L_KFE() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 5) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_fr_L_KFE& MotionTransforms::Type_fr_trunk_X_fr_L_KFE::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    (*this)(0, 0) = -sin_q_L_HFE;
    (*this)(0, 1) = -cos_q_L_HFE;
    (*this)(1, 0) = -sin_q_L_HAA * cos_q_L_HFE;
    (*this)(1, 1) = sin_q_L_HAA * sin_q_L_HFE;
    (*this)(1, 2) = cos_q_L_HAA;
    (*this)(2, 0) = -cos_q_L_HAA * cos_q_L_HFE;
    (*this)(2, 1) = cos_q_L_HAA * sin_q_L_HFE;
    (*this)(2, 2) = -sin_q_L_HAA;
    (*this)(3, 0) = ((tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA)) * cos_q_L_HFE;
    (*this)(3, 1) = ((ty_L_HAA * cos_q_L_HAA) - (tz_L_HAA * sin_q_L_HAA)) * sin_q_L_HFE;
    (*this)(3, 2) = (tx_L_KFE * cos_q_L_HFE) - (ty_L_HAA * sin_q_L_HAA) - (tz_L_HAA * cos_q_L_HAA) + tx_L_HFE;
    (*this)(3, 3) = -sin_q_L_HFE;
    (*this)(3, 4) = -cos_q_L_HFE;
    (*this)(4, 0) = ((tx_L_HFE * cos_q_L_HAA) - tz_L_HAA) * sin_q_L_HFE;
    (*this)(4, 1) = (((tx_L_HFE * cos_q_L_HAA) - tz_L_HAA) * cos_q_L_HFE) + (tx_L_KFE * cos_q_L_HAA);
    (*this)(4, 2) = -tx_L_KFE * sin_q_L_HAA * sin_q_L_HFE;
    (*this)(4, 3) = -sin_q_L_HAA * cos_q_L_HFE;
    (*this)(4, 4) = sin_q_L_HAA * sin_q_L_HFE;
    (*this)(4, 5) = cos_q_L_HAA;
    (*this)(5, 0) = (ty_L_HAA - (tx_L_HFE * sin_q_L_HAA)) * sin_q_L_HFE;
    (*this)(5, 1) = ((ty_L_HAA - (tx_L_HFE * sin_q_L_HAA)) * cos_q_L_HFE) - (tx_L_KFE * sin_q_L_HAA);
    (*this)(5, 2) = -tx_L_KFE * cos_q_L_HAA * sin_q_L_HFE;
    (*this)(5, 3) = -cos_q_L_HAA * cos_q_L_HFE;
    (*this)(5, 4) = cos_q_L_HAA * sin_q_L_HFE;
    (*this)(5, 5) = -sin_q_L_HAA;
    return *this;
}
MotionTransforms::Type_fr_trunk_X_R_foot::Type_fr_trunk_X_R_foot() {
    (*this)(0, 1) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 4) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_R_foot& MotionTransforms::Type_fr_trunk_X_R_foot::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(0, 0) = (cos_q_R_HFE * cos_q_R_KFE) - (sin_q_R_HFE * sin_q_R_KFE);
    (*this)(0, 2) = (cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HFE * cos_q_R_KFE);
    (*this)(1, 0) = (sin_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(1, 2) = (sin_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE) - (sin_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE);
    (*this)(2, 0) = (-cos_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(2, 1) = sin_q_R_HAA;
    (*this)(2, 2) = (cos_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE);
    (*this)(3, 0) = (((-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA)) * cos_q_R_HFE * sin_q_R_KFE) +
                    (((-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA)) * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(3, 1) = (-tx_R_foot * sin_q_R_HFE * sin_q_R_KFE) + (tx_R_foot * cos_q_R_HFE * cos_q_R_KFE) +
                    (tx_R_KFE * cos_q_R_HFE) + (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA) + tx_R_HFE;
    (*this)(3, 2) = (((-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA)) * sin_q_R_HFE * sin_q_R_KFE) +
                    (((tz_R_HAA * sin_q_R_HAA) + (ty_R_HAA * cos_q_R_HAA)) * cos_q_R_HFE * cos_q_R_KFE);
    (*this)(3, 3) = (cos_q_R_HFE * cos_q_R_KFE) - (sin_q_R_HFE * sin_q_R_KFE);
    (*this)(3, 5) = (cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HFE * cos_q_R_KFE);
    (*this)(4, 0) = (((tx_R_HFE * cos_q_R_HAA) - tz_R_HAA) * sin_q_R_HFE * sin_q_R_KFE) +
                    ((((tz_R_HAA - (tx_R_HFE * cos_q_R_HAA)) * cos_q_R_HFE) - (tx_R_KFE * cos_q_R_HAA)) * cos_q_R_KFE) -
                    (tx_R_foot * cos_q_R_HAA);
    (*this)(4, 1) = (tx_R_foot * sin_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) +
                    (tx_R_foot * sin_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE) + (tx_R_KFE * sin_q_R_HAA * sin_q_R_HFE);
    (*this)(4, 2) = ((((tz_R_HAA - (tx_R_HFE * cos_q_R_HAA)) * cos_q_R_HFE) - (tx_R_KFE * cos_q_R_HAA)) * sin_q_R_KFE) +
                    ((tz_R_HAA - (tx_R_HFE * cos_q_R_HAA)) * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(4, 3) = (sin_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(4, 4) = cos_q_R_HAA;
    (*this)(4, 5) = (sin_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE) - (sin_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE);
    (*this)(5, 0) = (((tx_R_HFE * sin_q_R_HAA) + ty_R_HAA) * sin_q_R_HFE * sin_q_R_KFE) +
                    (((((-tx_R_HFE * sin_q_R_HAA) - ty_R_HAA) * cos_q_R_HFE) - (tx_R_KFE * sin_q_R_HAA)) *
                     cos_q_R_KFE) - (tx_R_foot * sin_q_R_HAA);
    (*this)(5, 1) = (-tx_R_foot * cos_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) -
                    (tx_R_foot * cos_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE) - (tx_R_KFE * cos_q_R_HAA * sin_q_R_HFE);
    (*this)(5, 2) =
            (((((-tx_R_HFE * sin_q_R_HAA) - ty_R_HAA) * cos_q_R_HFE) - (tx_R_KFE * sin_q_R_HAA)) * sin_q_R_KFE) +
            (((-tx_R_HFE * sin_q_R_HAA) - ty_R_HAA) * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(5, 3) = (-cos_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(5, 4) = sin_q_R_HAA;
    (*this)(5, 5) = (cos_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE);
    return *this;
}
MotionTransforms::Type_fr_trunk_X_fr_R_HAA::Type_fr_trunk_X_fr_R_HAA() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = -ty_R_HAA;    // Maxima DSL: -_k__ty_R_HAA
    (*this)(3, 1) = -tz_R_HAA;    // Maxima DSL: -_k__tz_R_HAA
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = tz_R_HAA;    // Maxima DSL: _k__tz_R_HAA
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = -ty_R_HAA;    // Maxima DSL: -_k__ty_R_HAA
    (*this)(5, 3) = -1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_fr_R_HAA& MotionTransforms::Type_fr_trunk_X_fr_R_HAA::update(const state_t& q) {
    return *this;
}
MotionTransforms::Type_fr_trunk_X_fr_R_HFE::Type_fr_trunk_X_fr_R_HFE() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = -1.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = -1.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 4) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_fr_R_HFE& MotionTransforms::Type_fr_trunk_X_fr_R_HFE::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    (*this)(1, 0) = sin_q_R_HAA;
    (*this)(1, 2) = cos_q_R_HAA;
    (*this)(2, 0) = -cos_q_R_HAA;
    (*this)(2, 2) = sin_q_R_HAA;
    (*this)(3, 0) = (-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA);
    (*this)(3, 2) = (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA) + tx_R_HFE;
    (*this)(4, 1) = (tx_R_HFE * cos_q_R_HAA) - tz_R_HAA;
    (*this)(4, 3) = sin_q_R_HAA;
    (*this)(4, 5) = cos_q_R_HAA;
    (*this)(5, 1) = (tx_R_HFE * sin_q_R_HAA) + ty_R_HAA;
    (*this)(5, 3) = -cos_q_R_HAA;
    (*this)(5, 5) = sin_q_R_HAA;
    return *this;
}
MotionTransforms::Type_fr_trunk_X_fr_R_KFE::Type_fr_trunk_X_fr_R_KFE() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 5) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_fr_R_KFE& MotionTransforms::Type_fr_trunk_X_fr_R_KFE::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    (*this)(0, 0) = -sin_q_R_HFE;
    (*this)(0, 1) = -cos_q_R_HFE;
    (*this)(1, 0) = sin_q_R_HAA * cos_q_R_HFE;
    (*this)(1, 1) = -sin_q_R_HAA * sin_q_R_HFE;
    (*this)(1, 2) = cos_q_R_HAA;
    (*this)(2, 0) = -cos_q_R_HAA * cos_q_R_HFE;
    (*this)(2, 1) = cos_q_R_HAA * sin_q_R_HFE;
    (*this)(2, 2) = sin_q_R_HAA;
    (*this)(3, 0) = ((-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA)) * cos_q_R_HFE;
    (*this)(3, 1) = ((tz_R_HAA * sin_q_R_HAA) + (ty_R_HAA * cos_q_R_HAA)) * sin_q_R_HFE;
    (*this)(3, 2) = (tx_R_KFE * cos_q_R_HFE) + (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA) + tx_R_HFE;
    (*this)(3, 3) = -sin_q_R_HFE;
    (*this)(3, 4) = -cos_q_R_HFE;
    (*this)(4, 0) = ((tx_R_HFE * cos_q_R_HAA) - tz_R_HAA) * sin_q_R_HFE;
    (*this)(4, 1) = (((tx_R_HFE * cos_q_R_HAA) - tz_R_HAA) * cos_q_R_HFE) + (tx_R_KFE * cos_q_R_HAA);
    (*this)(4, 2) = tx_R_KFE * sin_q_R_HAA * sin_q_R_HFE;
    (*this)(4, 3) = sin_q_R_HAA * cos_q_R_HFE;
    (*this)(4, 4) = -sin_q_R_HAA * sin_q_R_HFE;
    (*this)(4, 5) = cos_q_R_HAA;
    (*this)(5, 0) = ((tx_R_HFE * sin_q_R_HAA) + ty_R_HAA) * sin_q_R_HFE;
    (*this)(5, 1) = (((tx_R_HFE * sin_q_R_HAA) + ty_R_HAA) * cos_q_R_HFE) + (tx_R_KFE * sin_q_R_HAA);
    (*this)(5, 2) = -tx_R_KFE * cos_q_R_HAA * sin_q_R_HFE;
    (*this)(5, 3) = -cos_q_R_HAA * cos_q_R_HFE;
    (*this)(5, 4) = cos_q_R_HAA * sin_q_R_HFE;
    (*this)(5, 5) = sin_q_R_HAA;
    return *this;
}
MotionTransforms::Type_fr_L_hip_X_fr_trunk::Type_fr_L_hip_X_fr_trunk() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = -tz_L_HAA;    // Maxima DSL: -_k__tz_L_HAA
    (*this)(5, 2) = ty_L_HAA;    // Maxima DSL: _k__ty_L_HAA
    (*this)(5, 3) = -1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_L_hip_X_fr_trunk& MotionTransforms::Type_fr_L_hip_X_fr_trunk::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    (*this)(0, 1) = -sin_q_L_HAA;
    (*this)(0, 2) = -cos_q_L_HAA;
    (*this)(1, 1) = -cos_q_L_HAA;
    (*this)(1, 2) = sin_q_L_HAA;
    (*this)(3, 0) = (tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA);
    (*this)(3, 4) = -sin_q_L_HAA;
    (*this)(3, 5) = -cos_q_L_HAA;
    (*this)(4, 0) = (ty_L_HAA * sin_q_L_HAA) + (tz_L_HAA * cos_q_L_HAA);
    (*this)(4, 4) = -cos_q_L_HAA;
    (*this)(4, 5) = sin_q_L_HAA;
    return *this;
}
MotionTransforms::Type_fr_trunk_X_fr_L_hip::Type_fr_trunk_X_fr_L_hip() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = -1.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = -1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = -tz_L_HAA;    // Maxima DSL: -_k__tz_L_HAA
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = ty_L_HAA;    // Maxima DSL: _k__ty_L_HAA
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_fr_L_hip& MotionTransforms::Type_fr_trunk_X_fr_L_hip::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    (*this)(1, 0) = -sin_q_L_HAA;
    (*this)(1, 1) = -cos_q_L_HAA;
    (*this)(2, 0) = -cos_q_L_HAA;
    (*this)(2, 1) = sin_q_L_HAA;
    (*this)(3, 0) = (tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA);
    (*this)(3, 1) = (ty_L_HAA * sin_q_L_HAA) + (tz_L_HAA * cos_q_L_HAA);
    (*this)(4, 3) = -sin_q_L_HAA;
    (*this)(4, 4) = -cos_q_L_HAA;
    (*this)(5, 3) = -cos_q_L_HAA;
    (*this)(5, 4) = sin_q_L_HAA;
    return *this;
}
MotionTransforms::Type_fr_L_thigh_X_fr_L_hip::Type_fr_L_thigh_X_fr_L_hip() {
    (*this)(0, 1) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = -1.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = -tx_L_HFE;    // Maxima DSL: -_k__tx_L_HFE
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = -1.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_L_thigh_X_fr_L_hip&
MotionTransforms::Type_fr_L_thigh_X_fr_L_hip::update(const state_t& q) {
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    (*this)(0, 0) = cos_q_L_HFE;
    (*this)(0, 2) = sin_q_L_HFE;
    (*this)(1, 0) = -sin_q_L_HFE;
    (*this)(1, 2) = cos_q_L_HFE;
    (*this)(3, 1) = -tx_L_HFE * sin_q_L_HFE;
    (*this)(3, 3) = cos_q_L_HFE;
    (*this)(3, 5) = sin_q_L_HFE;
    (*this)(4, 1) = -tx_L_HFE * cos_q_L_HFE;
    (*this)(4, 3) = -sin_q_L_HFE;
    (*this)(4, 5) = cos_q_L_HFE;
    return *this;
}
MotionTransforms::Type_fr_L_hip_X_fr_L_thigh::Type_fr_L_hip_X_fr_L_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(4, 5) = -1.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = -tx_L_HFE;    // Maxima DSL: -_k__tx_L_HFE
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_L_hip_X_fr_L_thigh&
MotionTransforms::Type_fr_L_hip_X_fr_L_thigh::update(const state_t& q) {
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    (*this)(0, 0) = cos_q_L_HFE;
    (*this)(0, 1) = -sin_q_L_HFE;
    (*this)(2, 0) = sin_q_L_HFE;
    (*this)(2, 1) = cos_q_L_HFE;
    (*this)(3, 3) = cos_q_L_HFE;
    (*this)(3, 4) = -sin_q_L_HFE;
    (*this)(4, 0) = -tx_L_HFE * sin_q_L_HFE;
    (*this)(4, 1) = -tx_L_HFE * cos_q_L_HFE;
    (*this)(5, 3) = sin_q_L_HFE;
    (*this)(5, 4) = cos_q_L_HFE;
    return *this;
}
MotionTransforms::Type_fr_L_shin_X_fr_L_thigh::Type_fr_L_shin_X_fr_L_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = -tx_L_KFE;    // Maxima DSL: -_k__tx_L_KFE
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const MotionTransforms::Type_fr_L_shin_X_fr_L_thigh&
MotionTransforms::Type_fr_L_shin_X_fr_L_thigh::update(const state_t& q) {
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(0, 0) = cos_q_L_KFE;
    (*this)(0, 1) = sin_q_L_KFE;
    (*this)(1, 0) = -sin_q_L_KFE;
    (*this)(1, 1) = cos_q_L_KFE;
    (*this)(3, 2) = tx_L_KFE * sin_q_L_KFE;
    (*this)(3, 3) = cos_q_L_KFE;
    (*this)(3, 4) = sin_q_L_KFE;
    (*this)(4, 2) = tx_L_KFE * cos_q_L_KFE;
    (*this)(4, 3) = -sin_q_L_KFE;
    (*this)(4, 4) = cos_q_L_KFE;
    return *this;
}
MotionTransforms::Type_fr_L_thigh_X_fr_L_shin::Type_fr_L_thigh_X_fr_L_shin() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = -tx_L_KFE;    // Maxima DSL: -_k__tx_L_KFE
    (*this)(4, 5) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const MotionTransforms::Type_fr_L_thigh_X_fr_L_shin&
MotionTransforms::Type_fr_L_thigh_X_fr_L_shin::update(const state_t& q) {
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(0, 0) = cos_q_L_KFE;
    (*this)(0, 1) = -sin_q_L_KFE;
    (*this)(1, 0) = sin_q_L_KFE;
    (*this)(1, 1) = cos_q_L_KFE;
    (*this)(3, 3) = cos_q_L_KFE;
    (*this)(3, 4) = -sin_q_L_KFE;
    (*this)(4, 3) = sin_q_L_KFE;
    (*this)(4, 4) = cos_q_L_KFE;
    (*this)(5, 0) = tx_L_KFE * sin_q_L_KFE;
    (*this)(5, 1) = tx_L_KFE * cos_q_L_KFE;
    return *this;
}
MotionTransforms::Type_fr_R_hip_X_fr_trunk::Type_fr_R_hip_X_fr_trunk() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = tz_R_HAA;    // Maxima DSL: _k__tz_R_HAA
    (*this)(5, 2) = -ty_R_HAA;    // Maxima DSL: -_k__ty_R_HAA
    (*this)(5, 3) = 1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_R_hip_X_fr_trunk& MotionTransforms::Type_fr_R_hip_X_fr_trunk::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    (*this)(0, 1) = sin_q_R_HAA;
    (*this)(0, 2) = -cos_q_R_HAA;
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(1, 2) = sin_q_R_HAA;
    (*this)(3, 0) = (-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA);
    (*this)(3, 4) = sin_q_R_HAA;
    (*this)(3, 5) = -cos_q_R_HAA;
    (*this)(4, 0) = (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA);
    (*this)(4, 4) = cos_q_R_HAA;
    (*this)(4, 5) = sin_q_R_HAA;
    return *this;
}
MotionTransforms::Type_fr_trunk_X_fr_R_hip::Type_fr_trunk_X_fr_R_hip() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = tz_R_HAA;    // Maxima DSL: _k__tz_R_HAA
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = -ty_R_HAA;    // Maxima DSL: -_k__ty_R_HAA
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_trunk_X_fr_R_hip& MotionTransforms::Type_fr_trunk_X_fr_R_hip::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    (*this)(1, 0) = sin_q_R_HAA;
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(2, 0) = -cos_q_R_HAA;
    (*this)(2, 1) = sin_q_R_HAA;
    (*this)(3, 0) = (-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA);
    (*this)(3, 1) = (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA);
    (*this)(4, 3) = sin_q_R_HAA;
    (*this)(4, 4) = cos_q_R_HAA;
    (*this)(5, 3) = -cos_q_R_HAA;
    (*this)(5, 4) = sin_q_R_HAA;
    return *this;
}
MotionTransforms::Type_fr_R_thigh_X_fr_R_hip::Type_fr_R_thigh_X_fr_R_hip() {
    (*this)(0, 1) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 1.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = tx_R_HFE;    // Maxima DSL: _k__tx_R_HFE
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 1.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_R_thigh_X_fr_R_hip&
MotionTransforms::Type_fr_R_thigh_X_fr_R_hip::update(const state_t& q) {
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    (*this)(0, 0) = cos_q_R_HFE;
    (*this)(0, 2) = -sin_q_R_HFE;
    (*this)(1, 0) = -sin_q_R_HFE;
    (*this)(1, 2) = -cos_q_R_HFE;
    (*this)(3, 1) = tx_R_HFE * sin_q_R_HFE;
    (*this)(3, 3) = cos_q_R_HFE;
    (*this)(3, 5) = -sin_q_R_HFE;
    (*this)(4, 1) = tx_R_HFE * cos_q_R_HFE;
    (*this)(4, 3) = -sin_q_R_HFE;
    (*this)(4, 5) = -cos_q_R_HFE;
    return *this;
}
MotionTransforms::Type_fr_R_hip_X_fr_R_thigh::Type_fr_R_hip_X_fr_R_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = 1.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(4, 5) = 1.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = tx_R_HFE;    // Maxima DSL: _k__tx_R_HFE
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_R_hip_X_fr_R_thigh&
MotionTransforms::Type_fr_R_hip_X_fr_R_thigh::update(const state_t& q) {
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    (*this)(0, 0) = cos_q_R_HFE;
    (*this)(0, 1) = -sin_q_R_HFE;
    (*this)(2, 0) = -sin_q_R_HFE;
    (*this)(2, 1) = -cos_q_R_HFE;
    (*this)(3, 3) = cos_q_R_HFE;
    (*this)(3, 4) = -sin_q_R_HFE;
    (*this)(4, 0) = tx_R_HFE * sin_q_R_HFE;
    (*this)(4, 1) = tx_R_HFE * cos_q_R_HFE;
    (*this)(5, 3) = -sin_q_R_HFE;
    (*this)(5, 4) = -cos_q_R_HFE;
    return *this;
}
MotionTransforms::Type_fr_R_shin_X_fr_R_thigh::Type_fr_R_shin_X_fr_R_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = -tx_R_KFE;    // Maxima DSL: -_k__tx_R_KFE
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const MotionTransforms::Type_fr_R_shin_X_fr_R_thigh&
MotionTransforms::Type_fr_R_shin_X_fr_R_thigh::update(const state_t& q) {
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(0, 0) = cos_q_R_KFE;
    (*this)(0, 1) = sin_q_R_KFE;
    (*this)(1, 0) = -sin_q_R_KFE;
    (*this)(1, 1) = cos_q_R_KFE;
    (*this)(3, 2) = tx_R_KFE * sin_q_R_KFE;
    (*this)(3, 3) = cos_q_R_KFE;
    (*this)(3, 4) = sin_q_R_KFE;
    (*this)(4, 2) = tx_R_KFE * cos_q_R_KFE;
    (*this)(4, 3) = -sin_q_R_KFE;
    (*this)(4, 4) = cos_q_R_KFE;
    return *this;
}
MotionTransforms::Type_fr_R_thigh_X_fr_R_shin::Type_fr_R_thigh_X_fr_R_shin() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = -tx_R_KFE;    // Maxima DSL: -_k__tx_R_KFE
    (*this)(4, 5) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const MotionTransforms::Type_fr_R_thigh_X_fr_R_shin&
MotionTransforms::Type_fr_R_thigh_X_fr_R_shin::update(const state_t& q) {
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(0, 0) = cos_q_R_KFE;
    (*this)(0, 1) = -sin_q_R_KFE;
    (*this)(1, 0) = sin_q_R_KFE;
    (*this)(1, 1) = cos_q_R_KFE;
    (*this)(3, 3) = cos_q_R_KFE;
    (*this)(3, 4) = -sin_q_R_KFE;
    (*this)(4, 3) = sin_q_R_KFE;
    (*this)(4, 4) = cos_q_R_KFE;
    (*this)(5, 0) = tx_R_KFE * sin_q_R_KFE;
    (*this)(5, 1) = tx_R_KFE * cos_q_R_KFE;
    return *this;
}

ForceTransforms::Type_fr_trunk_X_L_foot::Type_fr_trunk_X_L_foot() {
    (*this)(0, 1) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_L_foot& ForceTransforms::Type_fr_trunk_X_L_foot::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(0, 0) = (cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HFE * sin_q_L_KFE);
    (*this)(0, 2) = (cos_q_L_HFE * sin_q_L_KFE) + (sin_q_L_HFE * cos_q_L_KFE);
    (*this)(0, 3) = (((tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA)) * cos_q_L_HFE * sin_q_L_KFE) +
                    (((tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA)) * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(0, 4) = (-tx_L_foot * sin_q_L_HFE * sin_q_L_KFE) + (tx_L_foot * cos_q_L_HFE * cos_q_L_KFE) +
                    (tx_L_KFE * cos_q_L_HFE) - (ty_L_HAA * sin_q_L_HAA) - (tz_L_HAA * cos_q_L_HAA) + tx_L_HFE;
    (*this)(0, 5) = (((tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA)) * sin_q_L_HFE * sin_q_L_KFE) +
                    (((ty_L_HAA * cos_q_L_HAA) - (tz_L_HAA * sin_q_L_HAA)) * cos_q_L_HFE * cos_q_L_KFE);
    (*this)(1, 0) = (-sin_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(1, 1) = cos_q_L_HAA;
    (*this)(1, 2) = (sin_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    (*this)(1, 3) = (((tx_L_HFE * cos_q_L_HAA) - tz_L_HAA) * sin_q_L_HFE * sin_q_L_KFE) +
                    ((((tz_L_HAA - (tx_L_HFE * cos_q_L_HAA)) * cos_q_L_HFE) - (tx_L_KFE * cos_q_L_HAA)) * cos_q_L_KFE) -
                    (tx_L_foot * cos_q_L_HAA);
    (*this)(1, 4) = (-tx_L_foot * sin_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) -
                    (tx_L_foot * sin_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE) - (tx_L_KFE * sin_q_L_HAA * sin_q_L_HFE);
    (*this)(1, 5) = ((((tz_L_HAA - (tx_L_HFE * cos_q_L_HAA)) * cos_q_L_HFE) - (tx_L_KFE * cos_q_L_HAA)) * sin_q_L_KFE) +
                    ((tz_L_HAA - (tx_L_HFE * cos_q_L_HAA)) * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(2, 0) = (-cos_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(2, 1) = -sin_q_L_HAA;
    (*this)(2, 2) = (cos_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    (*this)(2, 3) = ((ty_L_HAA - (tx_L_HFE * sin_q_L_HAA)) * sin_q_L_HFE * sin_q_L_KFE) +
                    (((((tx_L_HFE * sin_q_L_HAA) - ty_L_HAA) * cos_q_L_HFE) + (tx_L_KFE * sin_q_L_HAA)) * cos_q_L_KFE) +
                    (tx_L_foot * sin_q_L_HAA);
    (*this)(2, 4) = (-tx_L_foot * cos_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) -
                    (tx_L_foot * cos_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE) - (tx_L_KFE * cos_q_L_HAA * sin_q_L_HFE);
    (*this)(2, 5) = (((((tx_L_HFE * sin_q_L_HAA) - ty_L_HAA) * cos_q_L_HFE) + (tx_L_KFE * sin_q_L_HAA)) * sin_q_L_KFE) +
                    (((tx_L_HFE * sin_q_L_HAA) - ty_L_HAA) * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(3, 3) = (cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HFE * sin_q_L_KFE);
    (*this)(3, 5) = (cos_q_L_HFE * sin_q_L_KFE) + (sin_q_L_HFE * cos_q_L_KFE);
    (*this)(4, 3) = (-sin_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(4, 4) = cos_q_L_HAA;
    (*this)(4, 5) = (sin_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    (*this)(5, 3) = (-cos_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(5, 4) = -sin_q_L_HAA;
    (*this)(5, 5) = (cos_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    return *this;
}
ForceTransforms::Type_fr_trunk_X_fr_L_HAA::Type_fr_trunk_X_fr_L_HAA() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = -1.0;
    (*this)(0, 3) = -ty_L_HAA;    // Maxima DSL: -_k__ty_L_HAA
    (*this)(0, 4) = tz_L_HAA;    // Maxima DSL: _k__tz_L_HAA
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = -1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = -tz_L_HAA;    // Maxima DSL: -_k__tz_L_HAA
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = ty_L_HAA;    // Maxima DSL: _k__ty_L_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = -1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = -1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = -1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_fr_L_HAA& ForceTransforms::Type_fr_trunk_X_fr_L_HAA::update(const state_t& q) {
    return *this;
}
ForceTransforms::Type_fr_trunk_X_fr_L_HFE::Type_fr_trunk_X_fr_L_HFE() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = -1.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = -1.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 4) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_fr_L_HFE& ForceTransforms::Type_fr_trunk_X_fr_L_HFE::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    (*this)(0, 3) = (tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA);
    (*this)(0, 5) = (-ty_L_HAA * sin_q_L_HAA) - (tz_L_HAA * cos_q_L_HAA) + tx_L_HFE;
    (*this)(1, 0) = -sin_q_L_HAA;
    (*this)(1, 2) = cos_q_L_HAA;
    (*this)(1, 4) = (tx_L_HFE * cos_q_L_HAA) - tz_L_HAA;
    (*this)(2, 0) = -cos_q_L_HAA;
    (*this)(2, 2) = -sin_q_L_HAA;
    (*this)(2, 4) = ty_L_HAA - (tx_L_HFE * sin_q_L_HAA);
    (*this)(4, 3) = -sin_q_L_HAA;
    (*this)(4, 5) = cos_q_L_HAA;
    (*this)(5, 3) = -cos_q_L_HAA;
    (*this)(5, 5) = -sin_q_L_HAA;
    return *this;
}
ForceTransforms::Type_fr_trunk_X_fr_L_KFE::Type_fr_trunk_X_fr_L_KFE() {
    (*this)(0, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_fr_L_KFE& ForceTransforms::Type_fr_trunk_X_fr_L_KFE::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    (*this)(0, 0) = -sin_q_L_HFE;
    (*this)(0, 1) = -cos_q_L_HFE;
    (*this)(0, 3) = ((tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA)) * cos_q_L_HFE;
    (*this)(0, 4) = ((ty_L_HAA * cos_q_L_HAA) - (tz_L_HAA * sin_q_L_HAA)) * sin_q_L_HFE;
    (*this)(0, 5) = (tx_L_KFE * cos_q_L_HFE) - (ty_L_HAA * sin_q_L_HAA) - (tz_L_HAA * cos_q_L_HAA) + tx_L_HFE;
    (*this)(1, 0) = -sin_q_L_HAA * cos_q_L_HFE;
    (*this)(1, 1) = sin_q_L_HAA * sin_q_L_HFE;
    (*this)(1, 2) = cos_q_L_HAA;
    (*this)(1, 3) = ((tx_L_HFE * cos_q_L_HAA) - tz_L_HAA) * sin_q_L_HFE;
    (*this)(1, 4) = (((tx_L_HFE * cos_q_L_HAA) - tz_L_HAA) * cos_q_L_HFE) + (tx_L_KFE * cos_q_L_HAA);
    (*this)(1, 5) = -tx_L_KFE * sin_q_L_HAA * sin_q_L_HFE;
    (*this)(2, 0) = -cos_q_L_HAA * cos_q_L_HFE;
    (*this)(2, 1) = cos_q_L_HAA * sin_q_L_HFE;
    (*this)(2, 2) = -sin_q_L_HAA;
    (*this)(2, 3) = (ty_L_HAA - (tx_L_HFE * sin_q_L_HAA)) * sin_q_L_HFE;
    (*this)(2, 4) = ((ty_L_HAA - (tx_L_HFE * sin_q_L_HAA)) * cos_q_L_HFE) - (tx_L_KFE * sin_q_L_HAA);
    (*this)(2, 5) = -tx_L_KFE * cos_q_L_HAA * sin_q_L_HFE;
    (*this)(3, 3) = -sin_q_L_HFE;
    (*this)(3, 4) = -cos_q_L_HFE;
    (*this)(4, 3) = -sin_q_L_HAA * cos_q_L_HFE;
    (*this)(4, 4) = sin_q_L_HAA * sin_q_L_HFE;
    (*this)(4, 5) = cos_q_L_HAA;
    (*this)(5, 3) = -cos_q_L_HAA * cos_q_L_HFE;
    (*this)(5, 4) = cos_q_L_HAA * sin_q_L_HFE;
    (*this)(5, 5) = -sin_q_L_HAA;
    return *this;
}
ForceTransforms::Type_fr_trunk_X_R_foot::Type_fr_trunk_X_R_foot() {
    (*this)(0, 1) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_R_foot& ForceTransforms::Type_fr_trunk_X_R_foot::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(0, 0) = (cos_q_R_HFE * cos_q_R_KFE) - (sin_q_R_HFE * sin_q_R_KFE);
    (*this)(0, 2) = (cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HFE * cos_q_R_KFE);
    (*this)(0, 3) = (((-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA)) * cos_q_R_HFE * sin_q_R_KFE) +
                    (((-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA)) * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(0, 4) = (-tx_R_foot * sin_q_R_HFE * sin_q_R_KFE) + (tx_R_foot * cos_q_R_HFE * cos_q_R_KFE) +
                    (tx_R_KFE * cos_q_R_HFE) + (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA) + tx_R_HFE;
    (*this)(0, 5) = (((-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA)) * sin_q_R_HFE * sin_q_R_KFE) +
                    (((tz_R_HAA * sin_q_R_HAA) + (ty_R_HAA * cos_q_R_HAA)) * cos_q_R_HFE * cos_q_R_KFE);
    (*this)(1, 0) = (sin_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(1, 2) = (sin_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE) - (sin_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE);
    (*this)(1, 3) = (((tx_R_HFE * cos_q_R_HAA) - tz_R_HAA) * sin_q_R_HFE * sin_q_R_KFE) +
                    ((((tz_R_HAA - (tx_R_HFE * cos_q_R_HAA)) * cos_q_R_HFE) - (tx_R_KFE * cos_q_R_HAA)) * cos_q_R_KFE) -
                    (tx_R_foot * cos_q_R_HAA);
    (*this)(1, 4) = (tx_R_foot * sin_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) +
                    (tx_R_foot * sin_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE) + (tx_R_KFE * sin_q_R_HAA * sin_q_R_HFE);
    (*this)(1, 5) = ((((tz_R_HAA - (tx_R_HFE * cos_q_R_HAA)) * cos_q_R_HFE) - (tx_R_KFE * cos_q_R_HAA)) * sin_q_R_KFE) +
                    ((tz_R_HAA - (tx_R_HFE * cos_q_R_HAA)) * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(2, 0) = (-cos_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(2, 1) = sin_q_R_HAA;
    (*this)(2, 2) = (cos_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE);
    (*this)(2, 3) = (((tx_R_HFE * sin_q_R_HAA) + ty_R_HAA) * sin_q_R_HFE * sin_q_R_KFE) +
                    (((((-tx_R_HFE * sin_q_R_HAA) - ty_R_HAA) * cos_q_R_HFE) - (tx_R_KFE * sin_q_R_HAA)) *
                     cos_q_R_KFE) - (tx_R_foot * sin_q_R_HAA);
    (*this)(2, 4) = (-tx_R_foot * cos_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) -
                    (tx_R_foot * cos_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE) - (tx_R_KFE * cos_q_R_HAA * sin_q_R_HFE);
    (*this)(2, 5) =
            (((((-tx_R_HFE * sin_q_R_HAA) - ty_R_HAA) * cos_q_R_HFE) - (tx_R_KFE * sin_q_R_HAA)) * sin_q_R_KFE) +
            (((-tx_R_HFE * sin_q_R_HAA) - ty_R_HAA) * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(3, 3) = (cos_q_R_HFE * cos_q_R_KFE) - (sin_q_R_HFE * sin_q_R_KFE);
    (*this)(3, 5) = (cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HFE * cos_q_R_KFE);
    (*this)(4, 3) = (sin_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(4, 4) = cos_q_R_HAA;
    (*this)(4, 5) = (sin_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE) - (sin_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE);
    (*this)(5, 3) = (-cos_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(5, 4) = sin_q_R_HAA;
    (*this)(5, 5) = (cos_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE);
    return *this;
}
ForceTransforms::Type_fr_trunk_X_fr_R_HAA::Type_fr_trunk_X_fr_R_HAA() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(0, 3) = -ty_R_HAA;    // Maxima DSL: -_k__ty_R_HAA
    (*this)(0, 4) = -tz_R_HAA;    // Maxima DSL: -_k__tz_R_HAA
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = tz_R_HAA;    // Maxima DSL: _k__tz_R_HAA
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = -ty_R_HAA;    // Maxima DSL: -_k__ty_R_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = -1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_fr_R_HAA& ForceTransforms::Type_fr_trunk_X_fr_R_HAA::update(const state_t& q) {
    return *this;
}
ForceTransforms::Type_fr_trunk_X_fr_R_HFE::Type_fr_trunk_X_fr_R_HFE() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = -1.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = -1.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 4) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_fr_R_HFE& ForceTransforms::Type_fr_trunk_X_fr_R_HFE::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    (*this)(0, 3) = (-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA);
    (*this)(0, 5) = (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA) + tx_R_HFE;
    (*this)(1, 0) = sin_q_R_HAA;
    (*this)(1, 2) = cos_q_R_HAA;
    (*this)(1, 4) = (tx_R_HFE * cos_q_R_HAA) - tz_R_HAA;
    (*this)(2, 0) = -cos_q_R_HAA;
    (*this)(2, 2) = sin_q_R_HAA;
    (*this)(2, 4) = (tx_R_HFE * sin_q_R_HAA) + ty_R_HAA;
    (*this)(4, 3) = sin_q_R_HAA;
    (*this)(4, 5) = cos_q_R_HAA;
    (*this)(5, 3) = -cos_q_R_HAA;
    (*this)(5, 5) = sin_q_R_HAA;
    return *this;
}
ForceTransforms::Type_fr_trunk_X_fr_R_KFE::Type_fr_trunk_X_fr_R_KFE() {
    (*this)(0, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_fr_R_KFE& ForceTransforms::Type_fr_trunk_X_fr_R_KFE::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    (*this)(0, 0) = -sin_q_R_HFE;
    (*this)(0, 1) = -cos_q_R_HFE;
    (*this)(0, 3) = ((-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA)) * cos_q_R_HFE;
    (*this)(0, 4) = ((tz_R_HAA * sin_q_R_HAA) + (ty_R_HAA * cos_q_R_HAA)) * sin_q_R_HFE;
    (*this)(0, 5) = (tx_R_KFE * cos_q_R_HFE) + (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA) + tx_R_HFE;
    (*this)(1, 0) = sin_q_R_HAA * cos_q_R_HFE;
    (*this)(1, 1) = -sin_q_R_HAA * sin_q_R_HFE;
    (*this)(1, 2) = cos_q_R_HAA;
    (*this)(1, 3) = ((tx_R_HFE * cos_q_R_HAA) - tz_R_HAA) * sin_q_R_HFE;
    (*this)(1, 4) = (((tx_R_HFE * cos_q_R_HAA) - tz_R_HAA) * cos_q_R_HFE) + (tx_R_KFE * cos_q_R_HAA);
    (*this)(1, 5) = tx_R_KFE * sin_q_R_HAA * sin_q_R_HFE;
    (*this)(2, 0) = -cos_q_R_HAA * cos_q_R_HFE;
    (*this)(2, 1) = cos_q_R_HAA * sin_q_R_HFE;
    (*this)(2, 2) = sin_q_R_HAA;
    (*this)(2, 3) = ((tx_R_HFE * sin_q_R_HAA) + ty_R_HAA) * sin_q_R_HFE;
    (*this)(2, 4) = (((tx_R_HFE * sin_q_R_HAA) + ty_R_HAA) * cos_q_R_HFE) + (tx_R_KFE * sin_q_R_HAA);
    (*this)(2, 5) = -tx_R_KFE * cos_q_R_HAA * sin_q_R_HFE;
    (*this)(3, 3) = -sin_q_R_HFE;
    (*this)(3, 4) = -cos_q_R_HFE;
    (*this)(4, 3) = sin_q_R_HAA * cos_q_R_HFE;
    (*this)(4, 4) = -sin_q_R_HAA * sin_q_R_HFE;
    (*this)(4, 5) = cos_q_R_HAA;
    (*this)(5, 3) = -cos_q_R_HAA * cos_q_R_HFE;
    (*this)(5, 4) = cos_q_R_HAA * sin_q_R_HFE;
    (*this)(5, 5) = sin_q_R_HAA;
    return *this;
}
ForceTransforms::Type_fr_L_hip_X_fr_trunk::Type_fr_L_hip_X_fr_trunk() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = -tz_L_HAA;    // Maxima DSL: -_k__tz_L_HAA
    (*this)(2, 5) = ty_L_HAA;    // Maxima DSL: _k__ty_L_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = -1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_L_hip_X_fr_trunk& ForceTransforms::Type_fr_L_hip_X_fr_trunk::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    (*this)(0, 1) = -sin_q_L_HAA;
    (*this)(0, 2) = -cos_q_L_HAA;
    (*this)(0, 3) = (tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA);
    (*this)(1, 1) = -cos_q_L_HAA;
    (*this)(1, 2) = sin_q_L_HAA;
    (*this)(1, 3) = (ty_L_HAA * sin_q_L_HAA) + (tz_L_HAA * cos_q_L_HAA);
    (*this)(3, 4) = -sin_q_L_HAA;
    (*this)(3, 5) = -cos_q_L_HAA;
    (*this)(4, 4) = -cos_q_L_HAA;
    (*this)(4, 5) = sin_q_L_HAA;
    return *this;
}
ForceTransforms::Type_fr_trunk_X_fr_L_hip::Type_fr_trunk_X_fr_L_hip() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = -1.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = -tz_L_HAA;    // Maxima DSL: -_k__tz_L_HAA
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = ty_L_HAA;    // Maxima DSL: _k__ty_L_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = -1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_fr_L_hip& ForceTransforms::Type_fr_trunk_X_fr_L_hip::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    (*this)(0, 3) = (tz_L_HAA * sin_q_L_HAA) - (ty_L_HAA * cos_q_L_HAA);
    (*this)(0, 4) = (ty_L_HAA * sin_q_L_HAA) + (tz_L_HAA * cos_q_L_HAA);
    (*this)(1, 0) = -sin_q_L_HAA;
    (*this)(1, 1) = -cos_q_L_HAA;
    (*this)(2, 0) = -cos_q_L_HAA;
    (*this)(2, 1) = sin_q_L_HAA;
    (*this)(4, 3) = -sin_q_L_HAA;
    (*this)(4, 4) = -cos_q_L_HAA;
    (*this)(5, 3) = -cos_q_L_HAA;
    (*this)(5, 4) = sin_q_L_HAA;
    return *this;
}
ForceTransforms::Type_fr_L_thigh_X_fr_L_hip::Type_fr_L_thigh_X_fr_L_hip() {
    (*this)(0, 1) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = -1.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = -tx_L_HFE;    // Maxima DSL: -_k__tx_L_HFE
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = -1.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_L_thigh_X_fr_L_hip&
ForceTransforms::Type_fr_L_thigh_X_fr_L_hip::update(const state_t& q) {
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    (*this)(0, 0) = cos_q_L_HFE;
    (*this)(0, 2) = sin_q_L_HFE;
    (*this)(0, 4) = -tx_L_HFE * sin_q_L_HFE;
    (*this)(1, 0) = -sin_q_L_HFE;
    (*this)(1, 2) = cos_q_L_HFE;
    (*this)(1, 4) = -tx_L_HFE * cos_q_L_HFE;
    (*this)(3, 3) = cos_q_L_HFE;
    (*this)(3, 5) = sin_q_L_HFE;
    (*this)(4, 3) = -sin_q_L_HFE;
    (*this)(4, 5) = cos_q_L_HFE;
    return *this;
}
ForceTransforms::Type_fr_L_hip_X_fr_L_thigh::Type_fr_L_hip_X_fr_L_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = -tx_L_HFE;    // Maxima DSL: -_k__tx_L_HFE
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(4, 5) = -1.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_L_hip_X_fr_L_thigh&
ForceTransforms::Type_fr_L_hip_X_fr_L_thigh::update(const state_t& q) {
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    (*this)(0, 0) = cos_q_L_HFE;
    (*this)(0, 1) = -sin_q_L_HFE;
    (*this)(1, 3) = -tx_L_HFE * sin_q_L_HFE;
    (*this)(1, 4) = -tx_L_HFE * cos_q_L_HFE;
    (*this)(2, 0) = sin_q_L_HFE;
    (*this)(2, 1) = cos_q_L_HFE;
    (*this)(3, 3) = cos_q_L_HFE;
    (*this)(3, 4) = -sin_q_L_HFE;
    (*this)(5, 3) = sin_q_L_HFE;
    (*this)(5, 4) = cos_q_L_HFE;
    return *this;
}
ForceTransforms::Type_fr_L_shin_X_fr_L_thigh::Type_fr_L_shin_X_fr_L_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = -tx_L_KFE;    // Maxima DSL: -_k__tx_L_KFE
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const ForceTransforms::Type_fr_L_shin_X_fr_L_thigh&
ForceTransforms::Type_fr_L_shin_X_fr_L_thigh::update(const state_t& q) {
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(0, 0) = cos_q_L_KFE;
    (*this)(0, 1) = sin_q_L_KFE;
    (*this)(0, 5) = tx_L_KFE * sin_q_L_KFE;
    (*this)(1, 0) = -sin_q_L_KFE;
    (*this)(1, 1) = cos_q_L_KFE;
    (*this)(1, 5) = tx_L_KFE * cos_q_L_KFE;
    (*this)(3, 3) = cos_q_L_KFE;
    (*this)(3, 4) = sin_q_L_KFE;
    (*this)(4, 3) = -sin_q_L_KFE;
    (*this)(4, 4) = cos_q_L_KFE;
    return *this;
}
ForceTransforms::Type_fr_L_thigh_X_fr_L_shin::Type_fr_L_thigh_X_fr_L_shin() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = -tx_L_KFE;    // Maxima DSL: -_k__tx_L_KFE
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const ForceTransforms::Type_fr_L_thigh_X_fr_L_shin&
ForceTransforms::Type_fr_L_thigh_X_fr_L_shin::update(const state_t& q) {
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(0, 0) = cos_q_L_KFE;
    (*this)(0, 1) = -sin_q_L_KFE;
    (*this)(1, 0) = sin_q_L_KFE;
    (*this)(1, 1) = cos_q_L_KFE;
    (*this)(2, 3) = tx_L_KFE * sin_q_L_KFE;
    (*this)(2, 4) = tx_L_KFE * cos_q_L_KFE;
    (*this)(3, 3) = cos_q_L_KFE;
    (*this)(3, 4) = -sin_q_L_KFE;
    (*this)(4, 3) = sin_q_L_KFE;
    (*this)(4, 4) = cos_q_L_KFE;
    return *this;
}
ForceTransforms::Type_fr_R_hip_X_fr_trunk::Type_fr_R_hip_X_fr_trunk() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = tz_R_HAA;    // Maxima DSL: _k__tz_R_HAA
    (*this)(2, 5) = -ty_R_HAA;    // Maxima DSL: -_k__ty_R_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_R_hip_X_fr_trunk& ForceTransforms::Type_fr_R_hip_X_fr_trunk::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    (*this)(0, 1) = sin_q_R_HAA;
    (*this)(0, 2) = -cos_q_R_HAA;
    (*this)(0, 3) = (-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA);
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(1, 2) = sin_q_R_HAA;
    (*this)(1, 3) = (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA);
    (*this)(3, 4) = sin_q_R_HAA;
    (*this)(3, 5) = -cos_q_R_HAA;
    (*this)(4, 4) = cos_q_R_HAA;
    (*this)(4, 5) = sin_q_R_HAA;
    return *this;
}
ForceTransforms::Type_fr_trunk_X_fr_R_hip::Type_fr_trunk_X_fr_R_hip() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = tz_R_HAA;    // Maxima DSL: _k__tz_R_HAA
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = -ty_R_HAA;    // Maxima DSL: -_k__ty_R_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_trunk_X_fr_R_hip& ForceTransforms::Type_fr_trunk_X_fr_R_hip::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    (*this)(0, 3) = (-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA);
    (*this)(0, 4) = (ty_R_HAA * sin_q_R_HAA) - (tz_R_HAA * cos_q_R_HAA);
    (*this)(1, 0) = sin_q_R_HAA;
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(2, 0) = -cos_q_R_HAA;
    (*this)(2, 1) = sin_q_R_HAA;
    (*this)(4, 3) = sin_q_R_HAA;
    (*this)(4, 4) = cos_q_R_HAA;
    (*this)(5, 3) = -cos_q_R_HAA;
    (*this)(5, 4) = sin_q_R_HAA;
    return *this;
}
ForceTransforms::Type_fr_R_thigh_X_fr_R_hip::Type_fr_R_thigh_X_fr_R_hip() {
    (*this)(0, 1) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 1.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = tx_R_HFE;    // Maxima DSL: _k__tx_R_HFE
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 1.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_R_thigh_X_fr_R_hip&
ForceTransforms::Type_fr_R_thigh_X_fr_R_hip::update(const state_t& q) {
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    (*this)(0, 0) = cos_q_R_HFE;
    (*this)(0, 2) = -sin_q_R_HFE;
    (*this)(0, 4) = tx_R_HFE * sin_q_R_HFE;
    (*this)(1, 0) = -sin_q_R_HFE;
    (*this)(1, 2) = -cos_q_R_HFE;
    (*this)(1, 4) = tx_R_HFE * cos_q_R_HFE;
    (*this)(3, 3) = cos_q_R_HFE;
    (*this)(3, 5) = -sin_q_R_HFE;
    (*this)(4, 3) = -sin_q_R_HFE;
    (*this)(4, 5) = -cos_q_R_HFE;
    return *this;
}
ForceTransforms::Type_fr_R_hip_X_fr_R_thigh::Type_fr_R_hip_X_fr_R_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = 1.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = tx_R_HFE;    // Maxima DSL: _k__tx_R_HFE
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(4, 5) = 1.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_R_hip_X_fr_R_thigh&
ForceTransforms::Type_fr_R_hip_X_fr_R_thigh::update(const state_t& q) {
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    (*this)(0, 0) = cos_q_R_HFE;
    (*this)(0, 1) = -sin_q_R_HFE;
    (*this)(1, 3) = tx_R_HFE * sin_q_R_HFE;
    (*this)(1, 4) = tx_R_HFE * cos_q_R_HFE;
    (*this)(2, 0) = -sin_q_R_HFE;
    (*this)(2, 1) = -cos_q_R_HFE;
    (*this)(3, 3) = cos_q_R_HFE;
    (*this)(3, 4) = -sin_q_R_HFE;
    (*this)(5, 3) = -sin_q_R_HFE;
    (*this)(5, 4) = -cos_q_R_HFE;
    return *this;
}
ForceTransforms::Type_fr_R_shin_X_fr_R_thigh::Type_fr_R_shin_X_fr_R_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = -tx_R_KFE;    // Maxima DSL: -_k__tx_R_KFE
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const ForceTransforms::Type_fr_R_shin_X_fr_R_thigh&
ForceTransforms::Type_fr_R_shin_X_fr_R_thigh::update(const state_t& q) {
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(0, 0) = cos_q_R_KFE;
    (*this)(0, 1) = sin_q_R_KFE;
    (*this)(0, 5) = tx_R_KFE * sin_q_R_KFE;
    (*this)(1, 0) = -sin_q_R_KFE;
    (*this)(1, 1) = cos_q_R_KFE;
    (*this)(1, 5) = tx_R_KFE * cos_q_R_KFE;
    (*this)(3, 3) = cos_q_R_KFE;
    (*this)(3, 4) = sin_q_R_KFE;
    (*this)(4, 3) = -sin_q_R_KFE;
    (*this)(4, 4) = cos_q_R_KFE;
    return *this;
}
ForceTransforms::Type_fr_R_thigh_X_fr_R_shin::Type_fr_R_thigh_X_fr_R_shin() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = -tx_R_KFE;    // Maxima DSL: -_k__tx_R_KFE
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const ForceTransforms::Type_fr_R_thigh_X_fr_R_shin&
ForceTransforms::Type_fr_R_thigh_X_fr_R_shin::update(const state_t& q) {
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(0, 0) = cos_q_R_KFE;
    (*this)(0, 1) = -sin_q_R_KFE;
    (*this)(1, 0) = sin_q_R_KFE;
    (*this)(1, 1) = cos_q_R_KFE;
    (*this)(2, 3) = tx_R_KFE * sin_q_R_KFE;
    (*this)(2, 4) = tx_R_KFE * cos_q_R_KFE;
    (*this)(3, 3) = cos_q_R_KFE;
    (*this)(3, 4) = -sin_q_R_KFE;
    (*this)(4, 3) = sin_q_R_KFE;
    (*this)(4, 4) = cos_q_R_KFE;
    return *this;
}

HomogeneousTransforms::Type_fr_trunk_X_L_foot::Type_fr_trunk_X_L_foot() {
    (*this)(0, 1) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_L_foot&
HomogeneousTransforms::Type_fr_trunk_X_L_foot::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(0, 0) = (cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HFE * sin_q_L_KFE);
    (*this)(0, 2) = (cos_q_L_HFE * sin_q_L_KFE) + (sin_q_L_HFE * cos_q_L_KFE);
    (*this)(0, 3) = (-tx_L_foot * cos_q_L_HFE * sin_q_L_KFE) - (tx_L_foot * sin_q_L_HFE * cos_q_L_KFE) -
                    (tx_L_KFE * sin_q_L_HFE);
    (*this)(1, 0) = (-sin_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(1, 1) = cos_q_L_HAA;
    (*this)(1, 2) = (sin_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (sin_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    (*this)(1, 3) = (tx_L_foot * sin_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE) -
                    (tx_L_foot * sin_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (tx_L_KFE * sin_q_L_HAA * cos_q_L_HFE) -
                    (tx_L_HFE * sin_q_L_HAA) + ty_L_HAA;
    (*this)(2, 0) = (-cos_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(2, 1) = -sin_q_L_HAA;
    (*this)(2, 2) = (cos_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (cos_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE);
    (*this)(2, 3) = (tx_L_foot * cos_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE) -
                    (tx_L_foot * cos_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (tx_L_KFE * cos_q_L_HAA * cos_q_L_HFE) -
                    (tx_L_HFE * cos_q_L_HAA) + tz_L_HAA;
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_fr_L_HAA::Type_fr_trunk_X_fr_L_HAA() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = -1.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = -1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = ty_L_HAA;    // Maxima DSL: _k__ty_L_HAA
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = tz_L_HAA;    // Maxima DSL: _k__tz_L_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_fr_L_HAA&
HomogeneousTransforms::Type_fr_trunk_X_fr_L_HAA::update(const state_t& q) {
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_fr_L_HFE::Type_fr_trunk_X_fr_L_HFE() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = -1.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_fr_L_HFE&
HomogeneousTransforms::Type_fr_trunk_X_fr_L_HFE::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    (*this)(1, 0) = -sin_q_L_HAA;
    (*this)(1, 2) = cos_q_L_HAA;
    (*this)(1, 3) = ty_L_HAA - (tx_L_HFE * sin_q_L_HAA);
    (*this)(2, 0) = -cos_q_L_HAA;
    (*this)(2, 2) = -sin_q_L_HAA;
    (*this)(2, 3) = tz_L_HAA - (tx_L_HFE * cos_q_L_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_fr_L_KFE::Type_fr_trunk_X_fr_L_KFE() {
    (*this)(0, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_fr_L_KFE&
HomogeneousTransforms::Type_fr_trunk_X_fr_L_KFE::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    (*this)(0, 0) = -sin_q_L_HFE;
    (*this)(0, 1) = -cos_q_L_HFE;
    (*this)(0, 3) = -tx_L_KFE * sin_q_L_HFE;
    (*this)(1, 0) = -sin_q_L_HAA * cos_q_L_HFE;
    (*this)(1, 1) = sin_q_L_HAA * sin_q_L_HFE;
    (*this)(1, 2) = cos_q_L_HAA;
    (*this)(1, 3) = (-tx_L_KFE * sin_q_L_HAA * cos_q_L_HFE) - (tx_L_HFE * sin_q_L_HAA) + ty_L_HAA;
    (*this)(2, 0) = -cos_q_L_HAA * cos_q_L_HFE;
    (*this)(2, 1) = cos_q_L_HAA * sin_q_L_HFE;
    (*this)(2, 2) = -sin_q_L_HAA;
    (*this)(2, 3) = (-tx_L_KFE * cos_q_L_HAA * cos_q_L_HFE) - (tx_L_HFE * cos_q_L_HAA) + tz_L_HAA;
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_R_foot::Type_fr_trunk_X_R_foot() {
    (*this)(0, 1) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_R_foot&
HomogeneousTransforms::Type_fr_trunk_X_R_foot::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(0, 0) = (cos_q_R_HFE * cos_q_R_KFE) - (sin_q_R_HFE * sin_q_R_KFE);
    (*this)(0, 2) = (cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HFE * cos_q_R_KFE);
    (*this)(0, 3) = (-tx_R_foot * cos_q_R_HFE * sin_q_R_KFE) - (tx_R_foot * sin_q_R_HFE * cos_q_R_KFE) -
                    (tx_R_KFE * sin_q_R_HFE);
    (*this)(1, 0) = (sin_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) + (sin_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(1, 2) = (sin_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE) - (sin_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE);
    (*this)(1, 3) = (-tx_R_foot * sin_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE) +
                    (tx_R_foot * sin_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE) + (tx_R_KFE * sin_q_R_HAA * cos_q_R_HFE) +
                    (tx_R_HFE * sin_q_R_HAA) + ty_R_HAA;
    (*this)(2, 0) = (-cos_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(2, 1) = sin_q_R_HAA;
    (*this)(2, 2) = (cos_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE) - (cos_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE);
    (*this)(2, 3) = (tx_R_foot * cos_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE) -
                    (tx_R_foot * cos_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE) - (tx_R_KFE * cos_q_R_HAA * cos_q_R_HFE) -
                    (tx_R_HFE * cos_q_R_HAA) + tz_R_HAA;
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_fr_R_HAA::Type_fr_trunk_X_fr_R_HAA() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = ty_R_HAA;    // Maxima DSL: _k__ty_R_HAA
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = tz_R_HAA;    // Maxima DSL: _k__tz_R_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_fr_R_HAA&
HomogeneousTransforms::Type_fr_trunk_X_fr_R_HAA::update(const state_t& q) {
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_fr_R_HFE::Type_fr_trunk_X_fr_R_HFE() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = -1.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_fr_R_HFE&
HomogeneousTransforms::Type_fr_trunk_X_fr_R_HFE::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    (*this)(1, 0) = sin_q_R_HAA;
    (*this)(1, 2) = cos_q_R_HAA;
    (*this)(1, 3) = (tx_R_HFE * sin_q_R_HAA) + ty_R_HAA;
    (*this)(2, 0) = -cos_q_R_HAA;
    (*this)(2, 2) = sin_q_R_HAA;
    (*this)(2, 3) = tz_R_HAA - (tx_R_HFE * cos_q_R_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_fr_R_KFE::Type_fr_trunk_X_fr_R_KFE() {
    (*this)(0, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_fr_R_KFE&
HomogeneousTransforms::Type_fr_trunk_X_fr_R_KFE::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    (*this)(0, 0) = -sin_q_R_HFE;
    (*this)(0, 1) = -cos_q_R_HFE;
    (*this)(0, 3) = -tx_R_KFE * sin_q_R_HFE;
    (*this)(1, 0) = sin_q_R_HAA * cos_q_R_HFE;
    (*this)(1, 1) = -sin_q_R_HAA * sin_q_R_HFE;
    (*this)(1, 2) = cos_q_R_HAA;
    (*this)(1, 3) = (tx_R_KFE * sin_q_R_HAA * cos_q_R_HFE) + (tx_R_HFE * sin_q_R_HAA) + ty_R_HAA;
    (*this)(2, 0) = -cos_q_R_HAA * cos_q_R_HFE;
    (*this)(2, 1) = cos_q_R_HAA * sin_q_R_HFE;
    (*this)(2, 2) = sin_q_R_HAA;
    (*this)(2, 3) = (-tx_R_KFE * cos_q_R_HAA * cos_q_R_HFE) - (tx_R_HFE * cos_q_R_HAA) + tz_R_HAA;
    return *this;
}
HomogeneousTransforms::Type_fr_L_hip_X_fr_trunk::Type_fr_L_hip_X_fr_trunk() {
    (*this)(0, 0) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_L_hip_X_fr_trunk&
HomogeneousTransforms::Type_fr_L_hip_X_fr_trunk::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    (*this)(0, 1) = -sin_q_L_HAA;
    (*this)(0, 2) = -cos_q_L_HAA;
    (*this)(0, 3) = (ty_L_HAA * sin_q_L_HAA) + (tz_L_HAA * cos_q_L_HAA);
    (*this)(1, 1) = -cos_q_L_HAA;
    (*this)(1, 2) = sin_q_L_HAA;
    (*this)(1, 3) = (ty_L_HAA * cos_q_L_HAA) - (tz_L_HAA * sin_q_L_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_fr_L_hip::Type_fr_trunk_X_fr_L_hip() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = -1.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = ty_L_HAA;    // Maxima DSL: _k__ty_L_HAA
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = tz_L_HAA;    // Maxima DSL: _k__tz_L_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_fr_L_hip&
HomogeneousTransforms::Type_fr_trunk_X_fr_L_hip::update(const state_t& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    (*this)(1, 0) = -sin_q_L_HAA;
    (*this)(1, 1) = -cos_q_L_HAA;
    (*this)(2, 0) = -cos_q_L_HAA;
    (*this)(2, 1) = sin_q_L_HAA;
    return *this;
}
HomogeneousTransforms::Type_fr_L_thigh_X_fr_L_hip::Type_fr_L_thigh_X_fr_L_hip() {
    (*this)(0, 1) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = -1.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_L_thigh_X_fr_L_hip&
HomogeneousTransforms::Type_fr_L_thigh_X_fr_L_hip::update(const state_t& q) {
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    (*this)(0, 0) = cos_q_L_HFE;
    (*this)(0, 2) = sin_q_L_HFE;
    (*this)(0, 3) = -tx_L_HFE * cos_q_L_HFE;
    (*this)(1, 0) = -sin_q_L_HFE;
    (*this)(1, 2) = cos_q_L_HFE;
    (*this)(1, 3) = tx_L_HFE * sin_q_L_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_L_hip_X_fr_L_thigh::Type_fr_L_hip_X_fr_L_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = tx_L_HFE;    // Maxima DSL: _k__tx_L_HFE
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_L_hip_X_fr_L_thigh&
HomogeneousTransforms::Type_fr_L_hip_X_fr_L_thigh::update(const state_t& q) {
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    (*this)(0, 0) = cos_q_L_HFE;
    (*this)(0, 1) = -sin_q_L_HFE;
    (*this)(2, 0) = sin_q_L_HFE;
    (*this)(2, 1) = cos_q_L_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_L_shin_X_fr_L_thigh::Type_fr_L_shin_X_fr_L_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_L_shin_X_fr_L_thigh&
HomogeneousTransforms::Type_fr_L_shin_X_fr_L_thigh::update(const state_t& q) {
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(0, 0) = cos_q_L_KFE;
    (*this)(0, 1) = sin_q_L_KFE;
    (*this)(0, 3) = -tx_L_KFE * cos_q_L_KFE;
    (*this)(1, 0) = -sin_q_L_KFE;
    (*this)(1, 1) = cos_q_L_KFE;
    (*this)(1, 3) = tx_L_KFE * sin_q_L_KFE;
    return *this;
}
HomogeneousTransforms::Type_fr_L_thigh_X_fr_L_shin::Type_fr_L_thigh_X_fr_L_shin() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = tx_L_KFE;    // Maxima DSL: _k__tx_L_KFE
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_L_thigh_X_fr_L_shin&
HomogeneousTransforms::Type_fr_L_thigh_X_fr_L_shin::update(const state_t& q) {
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(0, 0) = cos_q_L_KFE;
    (*this)(0, 1) = -sin_q_L_KFE;
    (*this)(1, 0) = sin_q_L_KFE;
    (*this)(1, 1) = cos_q_L_KFE;
    return *this;
}
HomogeneousTransforms::Type_fr_R_hip_X_fr_trunk::Type_fr_R_hip_X_fr_trunk() {
    (*this)(0, 0) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(2, 0) = 1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_R_hip_X_fr_trunk&
HomogeneousTransforms::Type_fr_R_hip_X_fr_trunk::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    (*this)(0, 1) = sin_q_R_HAA;
    (*this)(0, 2) = -cos_q_R_HAA;
    (*this)(0, 3) = (tz_R_HAA * cos_q_R_HAA) - (ty_R_HAA * sin_q_R_HAA);
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(1, 2) = sin_q_R_HAA;
    (*this)(1, 3) = (-tz_R_HAA * sin_q_R_HAA) - (ty_R_HAA * cos_q_R_HAA);
    return *this;
}
HomogeneousTransforms::Type_fr_trunk_X_fr_R_hip::Type_fr_trunk_X_fr_R_hip() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = ty_R_HAA;    // Maxima DSL: _k__ty_R_HAA
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = tz_R_HAA;    // Maxima DSL: _k__tz_R_HAA
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_trunk_X_fr_R_hip&
HomogeneousTransforms::Type_fr_trunk_X_fr_R_hip::update(const state_t& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    (*this)(1, 0) = sin_q_R_HAA;
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(2, 0) = -cos_q_R_HAA;
    (*this)(2, 1) = sin_q_R_HAA;
    return *this;
}
HomogeneousTransforms::Type_fr_R_thigh_X_fr_R_hip::Type_fr_R_thigh_X_fr_R_hip() {
    (*this)(0, 1) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 1.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_R_thigh_X_fr_R_hip&
HomogeneousTransforms::Type_fr_R_thigh_X_fr_R_hip::update(const state_t& q) {
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    (*this)(0, 0) = cos_q_R_HFE;
    (*this)(0, 2) = -sin_q_R_HFE;
    (*this)(0, 3) = -tx_R_HFE * cos_q_R_HFE;
    (*this)(1, 0) = -sin_q_R_HFE;
    (*this)(1, 2) = -cos_q_R_HFE;
    (*this)(1, 3) = tx_R_HFE * sin_q_R_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_R_hip_X_fr_R_thigh::Type_fr_R_hip_X_fr_R_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = tx_R_HFE;    // Maxima DSL: _k__tx_R_HFE
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = 1.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_R_hip_X_fr_R_thigh&
HomogeneousTransforms::Type_fr_R_hip_X_fr_R_thigh::update(const state_t& q) {
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    (*this)(0, 0) = cos_q_R_HFE;
    (*this)(0, 1) = -sin_q_R_HFE;
    (*this)(2, 0) = -sin_q_R_HFE;
    (*this)(2, 1) = -cos_q_R_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_R_shin_X_fr_R_thigh::Type_fr_R_shin_X_fr_R_thigh() {
    (*this)(0, 2) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_R_shin_X_fr_R_thigh&
HomogeneousTransforms::Type_fr_R_shin_X_fr_R_thigh::update(const state_t& q) {
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(0, 0) = cos_q_R_KFE;
    (*this)(0, 1) = sin_q_R_KFE;
    (*this)(0, 3) = -tx_R_KFE * cos_q_R_KFE;
    (*this)(1, 0) = -sin_q_R_KFE;
    (*this)(1, 1) = cos_q_R_KFE;
    (*this)(1, 3) = tx_R_KFE * sin_q_R_KFE;
    return *this;
}
HomogeneousTransforms::Type_fr_R_thigh_X_fr_R_shin::Type_fr_R_thigh_X_fr_R_shin() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = tx_R_KFE;    // Maxima DSL: _k__tx_R_KFE
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_R_thigh_X_fr_R_shin&
HomogeneousTransforms::Type_fr_R_thigh_X_fr_R_shin::update(const state_t& q) {
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(0, 0) = cos_q_R_KFE;
    (*this)(0, 1) = -sin_q_R_KFE;
    (*this)(1, 0) = sin_q_R_KFE;
    (*this)(1, 1) = cos_q_R_KFE;
    return *this;
}

