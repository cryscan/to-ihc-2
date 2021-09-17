#include "transforms.h"

using namespace Hopper::rcg;

// Constructors

MotionTransforms::MotionTransforms()
        : fr_u0_X_body(),
          fr_u0_X_knee(),
          fr_u0_X_foot(),
          fr_u0_X_fr_BH(),
          fr_u0_X_fr_BX(),
          fr_u0_X_fr_HFE(),
          fr_u0_X_fr_KFE(),
          fr_u1_X_fr_u0(),
          fr_u0_X_fr_u1(),
          fr_u2_X_fr_u1(),
          fr_u1_X_fr_u2(),
          fr_body_X_fr_u2(),
          fr_u2_X_fr_body(),
          fr_leg_X_fr_body(),
          fr_body_X_fr_leg() {}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles) {
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
        : fr_u0_X_body(),
          fr_u0_X_knee(),
          fr_u0_X_foot(),
          fr_u0_X_fr_BH(),
          fr_u0_X_fr_BX(),
          fr_u0_X_fr_HFE(),
          fr_u0_X_fr_KFE(),
          fr_u1_X_fr_u0(),
          fr_u0_X_fr_u1(),
          fr_u2_X_fr_u1(),
          fr_u1_X_fr_u2(),
          fr_body_X_fr_u2(),
          fr_u2_X_fr_body(),
          fr_leg_X_fr_body(),
          fr_body_X_fr_leg() {}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles) {
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
        : fr_u0_X_body(),
          fr_u0_X_knee(),
          fr_u0_X_foot(),
          fr_u0_X_fr_BH(),
          fr_u0_X_fr_BX(),
          fr_u0_X_fr_HFE(),
          fr_u0_X_fr_KFE(),
          fr_u1_X_fr_u0(),
          fr_u0_X_fr_u1(),
          fr_u2_X_fr_u1(),
          fr_u1_X_fr_u2(),
          fr_body_X_fr_u2(),
          fr_u2_X_fr_body(),
          fr_leg_X_fr_body(),
          fr_body_X_fr_leg() {}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles) {
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_u0_X_body::Type_fr_u0_X_body() {
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
    (*this)(3, 5) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(4, 5) = -1.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_u0_X_body& MotionTransforms::Type_fr_u0_X_body::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = sin_q_HFE;
    (*this)(0, 1) = cos_q_HFE;
    (*this)(2, 0) = -cos_q_HFE;
    (*this)(2, 1) = sin_q_HFE;
    (*this)(3, 2) = q(BH) + tz_BH;
    (*this)(3, 3) = sin_q_HFE;
    (*this)(3, 4) = cos_q_HFE;
    (*this)(4, 0) = ((q(BH) + tz_BH) * sin_q_HFE) + (q(BX) * cos_q_HFE);
    (*this)(4, 1) = ((q(BH) + tz_BH) * cos_q_HFE) - (q(BX) * sin_q_HFE);
    (*this)(5, 2) = -q(BX);
    (*this)(5, 3) = -cos_q_HFE;
    (*this)(5, 4) = sin_q_HFE;
    return *this;
}
MotionTransforms::Type_fr_u0_X_knee::Type_fr_u0_X_knee() {
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
    (*this)(3, 5) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(4, 5) = -1.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_u0_X_knee& MotionTransforms::Type_fr_u0_X_knee::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(0, 1) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(2, 0) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(2, 1) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(3, 2) = (-tx_KFE * cos_q_HFE) + q(BH) + tz_BH;
    (*this)(3, 3) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(3, 4) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(4, 0) = (((-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE) * sin_q_KFE) +
                    ((((q(BH) + tz_BH) * sin_q_HFE) + (q(BX) * cos_q_HFE)) * cos_q_KFE);
    (*this)(4, 1) = ((((-q(BH) - tz_BH) * sin_q_HFE) - (q(BX) * cos_q_HFE)) * sin_q_KFE) +
                    (((-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE) * cos_q_KFE);
    (*this)(5, 2) = (-tx_KFE * sin_q_HFE) - q(BX);
    (*this)(5, 3) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(5, 4) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    return *this;
}
MotionTransforms::Type_fr_u0_X_foot::Type_fr_u0_X_foot() {
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
    (*this)(3, 5) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(4, 5) = -1.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_u0_X_foot& MotionTransforms::Type_fr_u0_X_foot::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(0, 1) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(2, 0) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(2, 1) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(3, 2) =
            (tx_foot * sin_q_HFE * sin_q_KFE) - (tx_foot * cos_q_HFE * cos_q_KFE) - (tx_KFE * cos_q_HFE) + q(BH) +
            tz_BH;
    (*this)(3, 3) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(3, 4) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(4, 0) = (((-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE) * sin_q_KFE) +
                    ((((q(BH) + tz_BH) * sin_q_HFE) + (q(BX) * cos_q_HFE)) * cos_q_KFE);
    (*this)(4, 1) = ((((-q(BH) - tz_BH) * sin_q_HFE) - (q(BX) * cos_q_HFE)) * sin_q_KFE) +
                    (((-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE) * cos_q_KFE) - tx_foot;
    (*this)(5, 2) =
            (-tx_foot * cos_q_HFE * sin_q_KFE) - (tx_foot * sin_q_HFE * cos_q_KFE) - (tx_KFE * sin_q_HFE) - q(BX);
    (*this)(5, 3) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(5, 4) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    return *this;
}
MotionTransforms::Type_fr_u0_X_fr_BH::Type_fr_u0_X_fr_BH() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
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
    (*this)(3, 1) = -tz_BH;    // Maxima DSL: -_k__tz_BH
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = tz_BH;    // Maxima DSL: _k__tz_BH
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const MotionTransforms::Type_fr_u0_X_fr_BH& MotionTransforms::Type_fr_u0_X_fr_BH::update(const state_t& q) {
    return *this;
}
MotionTransforms::Type_fr_u0_X_fr_BX::Type_fr_u0_X_fr_BX() {
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
    (*this)(3, 0) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
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

const MotionTransforms::Type_fr_u0_X_fr_BX& MotionTransforms::Type_fr_u0_X_fr_BX::update(const state_t& q) {
    (*this)(3, 1) = -q(BH) - tz_BH;
    (*this)(4, 2) = q(BH) + tz_BH;
    return *this;
}
MotionTransforms::Type_fr_u0_X_fr_HFE::Type_fr_u0_X_fr_HFE() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 1.0;
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
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 1.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(4, 5) = -1.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 3) = -1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_u0_X_fr_HFE& MotionTransforms::Type_fr_u0_X_fr_HFE::update(const state_t& q) {
    (*this)(3, 2) = q(BH) + tz_BH;
    (*this)(4, 0) = q(BX);
    (*this)(4, 1) = q(BH) + tz_BH;
    (*this)(5, 2) = -q(BX);
    return *this;
}
MotionTransforms::Type_fr_u0_X_fr_KFE::Type_fr_u0_X_fr_KFE() {
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
    (*this)(3, 5) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 0.0;
    (*this)(4, 5) = -1.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_u0_X_fr_KFE& MotionTransforms::Type_fr_u0_X_fr_KFE::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = sin_q_HFE;
    (*this)(0, 1) = cos_q_HFE;
    (*this)(2, 0) = -cos_q_HFE;
    (*this)(2, 1) = sin_q_HFE;
    (*this)(3, 2) = (-tx_KFE * cos_q_HFE) + q(BH) + tz_BH;
    (*this)(3, 3) = sin_q_HFE;
    (*this)(3, 4) = cos_q_HFE;
    (*this)(4, 0) = ((q(BH) + tz_BH) * sin_q_HFE) + (q(BX) * cos_q_HFE);
    (*this)(4, 1) = (-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE;
    (*this)(5, 2) = (-tx_KFE * sin_q_HFE) - q(BX);
    (*this)(5, 3) = -cos_q_HFE;
    (*this)(5, 4) = sin_q_HFE;
    return *this;
}
MotionTransforms::Type_fr_u1_X_fr_u0::Type_fr_u1_X_fr_u0() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
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
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const MotionTransforms::Type_fr_u1_X_fr_u0& MotionTransforms::Type_fr_u1_X_fr_u0::update(const state_t& q) {
    (*this)(3, 1) = q(BH) + tz_BH;
    (*this)(4, 0) = -q(BH) - tz_BH;
    return *this;
}
MotionTransforms::Type_fr_u0_X_fr_u1::Type_fr_u0_X_fr_u1() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
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
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const MotionTransforms::Type_fr_u0_X_fr_u1& MotionTransforms::Type_fr_u0_X_fr_u1::update(const state_t& q) {
    (*this)(3, 1) = -q(BH) - tz_BH;
    (*this)(4, 0) = q(BH) + tz_BH;
    return *this;
}
MotionTransforms::Type_fr_u2_X_fr_u1::Type_fr_u2_X_fr_u1() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = -1.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = 1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = -1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_u2_X_fr_u1& MotionTransforms::Type_fr_u2_X_fr_u1::update(const state_t& q) {
    (*this)(3, 1) = q(BX);
    (*this)(4, 2) = q(BX);
    return *this;
}
MotionTransforms::Type_fr_u1_X_fr_u2::Type_fr_u1_X_fr_u2() {
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
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 1.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = -1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const MotionTransforms::Type_fr_u1_X_fr_u2& MotionTransforms::Type_fr_u1_X_fr_u2::update(const state_t& q) {
    (*this)(4, 0) = q(BX);
    (*this)(5, 1) = q(BX);
    return *this;
}
MotionTransforms::Type_fr_body_X_fr_u2::Type_fr_body_X_fr_u2() {
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

const MotionTransforms::Type_fr_body_X_fr_u2& MotionTransforms::Type_fr_body_X_fr_u2::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = cos_q_HFE;
    (*this)(0, 2) = sin_q_HFE;
    (*this)(1, 0) = -sin_q_HFE;
    (*this)(1, 2) = cos_q_HFE;
    (*this)(3, 3) = cos_q_HFE;
    (*this)(3, 5) = sin_q_HFE;
    (*this)(4, 3) = -sin_q_HFE;
    (*this)(4, 5) = cos_q_HFE;
    return *this;
}
MotionTransforms::Type_fr_u2_X_fr_body::Type_fr_u2_X_fr_body() {
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

const MotionTransforms::Type_fr_u2_X_fr_body& MotionTransforms::Type_fr_u2_X_fr_body::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = cos_q_HFE;
    (*this)(0, 1) = -sin_q_HFE;
    (*this)(2, 0) = sin_q_HFE;
    (*this)(2, 1) = cos_q_HFE;
    (*this)(3, 3) = cos_q_HFE;
    (*this)(3, 4) = -sin_q_HFE;
    (*this)(5, 3) = sin_q_HFE;
    (*this)(5, 4) = cos_q_HFE;
    return *this;
}
MotionTransforms::Type_fr_leg_X_fr_body::Type_fr_leg_X_fr_body() {
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
    (*this)(5, 1) = -tx_KFE;    // Maxima DSL: -_k__tx_KFE
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const MotionTransforms::Type_fr_leg_X_fr_body& MotionTransforms::Type_fr_leg_X_fr_body::update(const state_t& q) {
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = cos_q_KFE;
    (*this)(0, 1) = sin_q_KFE;
    (*this)(1, 0) = -sin_q_KFE;
    (*this)(1, 1) = cos_q_KFE;
    (*this)(3, 2) = tx_KFE * sin_q_KFE;
    (*this)(3, 3) = cos_q_KFE;
    (*this)(3, 4) = sin_q_KFE;
    (*this)(4, 2) = tx_KFE * cos_q_KFE;
    (*this)(4, 3) = -sin_q_KFE;
    (*this)(4, 4) = cos_q_KFE;
    return *this;
}
MotionTransforms::Type_fr_body_X_fr_leg::Type_fr_body_X_fr_leg() {
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
    (*this)(4, 2) = -tx_KFE;    // Maxima DSL: -_k__tx_KFE
    (*this)(4, 5) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const MotionTransforms::Type_fr_body_X_fr_leg& MotionTransforms::Type_fr_body_X_fr_leg::update(const state_t& q) {
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = cos_q_KFE;
    (*this)(0, 1) = -sin_q_KFE;
    (*this)(1, 0) = sin_q_KFE;
    (*this)(1, 1) = cos_q_KFE;
    (*this)(3, 3) = cos_q_KFE;
    (*this)(3, 4) = -sin_q_KFE;
    (*this)(4, 3) = sin_q_KFE;
    (*this)(4, 4) = cos_q_KFE;
    (*this)(5, 0) = tx_KFE * sin_q_KFE;
    (*this)(5, 1) = tx_KFE * cos_q_KFE;
    return *this;
}

ForceTransforms::Type_fr_u0_X_body::Type_fr_u0_X_body() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
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

const ForceTransforms::Type_fr_u0_X_body& ForceTransforms::Type_fr_u0_X_body::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = sin_q_HFE;
    (*this)(0, 1) = cos_q_HFE;
    (*this)(0, 5) = q(BH) + tz_BH;
    (*this)(1, 3) = ((q(BH) + tz_BH) * sin_q_HFE) + (q(BX) * cos_q_HFE);
    (*this)(1, 4) = ((q(BH) + tz_BH) * cos_q_HFE) - (q(BX) * sin_q_HFE);
    (*this)(2, 0) = -cos_q_HFE;
    (*this)(2, 1) = sin_q_HFE;
    (*this)(2, 5) = -q(BX);
    (*this)(3, 3) = sin_q_HFE;
    (*this)(3, 4) = cos_q_HFE;
    (*this)(5, 3) = -cos_q_HFE;
    (*this)(5, 4) = sin_q_HFE;
    return *this;
}
ForceTransforms::Type_fr_u0_X_knee::Type_fr_u0_X_knee() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
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

const ForceTransforms::Type_fr_u0_X_knee& ForceTransforms::Type_fr_u0_X_knee::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(0, 1) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(0, 5) = (-tx_KFE * cos_q_HFE) + q(BH) + tz_BH;
    (*this)(1, 3) = (((-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE) * sin_q_KFE) +
                    ((((q(BH) + tz_BH) * sin_q_HFE) + (q(BX) * cos_q_HFE)) * cos_q_KFE);
    (*this)(1, 4) = ((((-q(BH) - tz_BH) * sin_q_HFE) - (q(BX) * cos_q_HFE)) * sin_q_KFE) +
                    (((-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE) * cos_q_KFE);
    (*this)(2, 0) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(2, 1) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(2, 5) = (-tx_KFE * sin_q_HFE) - q(BX);
    (*this)(3, 3) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(3, 4) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(5, 3) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(5, 4) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    return *this;
}
ForceTransforms::Type_fr_u0_X_foot::Type_fr_u0_X_foot() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
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

const ForceTransforms::Type_fr_u0_X_foot& ForceTransforms::Type_fr_u0_X_foot::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(0, 1) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(0, 5) =
            (tx_foot * sin_q_HFE * sin_q_KFE) - (tx_foot * cos_q_HFE * cos_q_KFE) - (tx_KFE * cos_q_HFE) + q(BH) +
            tz_BH;
    (*this)(1, 3) = (((-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE) * sin_q_KFE) +
                    ((((q(BH) + tz_BH) * sin_q_HFE) + (q(BX) * cos_q_HFE)) * cos_q_KFE);
    (*this)(1, 4) = ((((-q(BH) - tz_BH) * sin_q_HFE) - (q(BX) * cos_q_HFE)) * sin_q_KFE) +
                    (((-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE) * cos_q_KFE) - tx_foot;
    (*this)(2, 0) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(2, 1) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(2, 5) =
            (-tx_foot * cos_q_HFE * sin_q_KFE) - (tx_foot * sin_q_HFE * cos_q_KFE) - (tx_KFE * sin_q_HFE) - q(BX);
    (*this)(3, 3) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(3, 4) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(5, 3) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(5, 4) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    return *this;
}
ForceTransforms::Type_fr_u0_X_fr_BH::Type_fr_u0_X_fr_BH() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = -tz_BH;    // Maxima DSL: -_k__tz_BH
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = tz_BH;    // Maxima DSL: _k__tz_BH
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
    (*this)(3, 3) = 1.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const ForceTransforms::Type_fr_u0_X_fr_BH& ForceTransforms::Type_fr_u0_X_fr_BH::update(const state_t& q) {
    return *this;
}
ForceTransforms::Type_fr_u0_X_fr_BX::Type_fr_u0_X_fr_BX() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
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

const ForceTransforms::Type_fr_u0_X_fr_BX& ForceTransforms::Type_fr_u0_X_fr_BX::update(const state_t& q) {
    (*this)(0, 4) = -q(BH) - tz_BH;
    (*this)(1, 5) = q(BH) + tz_BH;
    return *this;
}
ForceTransforms::Type_fr_u0_X_fr_HFE::Type_fr_u0_X_fr_HFE() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 1.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 0.0;
    (*this)(3, 4) = 1.0;
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
    (*this)(5, 3) = -1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_u0_X_fr_HFE& ForceTransforms::Type_fr_u0_X_fr_HFE::update(const state_t& q) {
    (*this)(0, 5) = q(BH) + tz_BH;
    (*this)(1, 3) = q(BX);
    (*this)(1, 4) = q(BH) + tz_BH;
    (*this)(2, 5) = -q(BX);
    return *this;
}
ForceTransforms::Type_fr_u0_X_fr_KFE::Type_fr_u0_X_fr_KFE() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
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

const ForceTransforms::Type_fr_u0_X_fr_KFE& ForceTransforms::Type_fr_u0_X_fr_KFE::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = sin_q_HFE;
    (*this)(0, 1) = cos_q_HFE;
    (*this)(0, 5) = (-tx_KFE * cos_q_HFE) + q(BH) + tz_BH;
    (*this)(1, 3) = ((q(BH) + tz_BH) * sin_q_HFE) + (q(BX) * cos_q_HFE);
    (*this)(1, 4) = (-q(BX) * sin_q_HFE) + ((q(BH) + tz_BH) * cos_q_HFE) - tx_KFE;
    (*this)(2, 0) = -cos_q_HFE;
    (*this)(2, 1) = sin_q_HFE;
    (*this)(2, 5) = (-tx_KFE * sin_q_HFE) - q(BX);
    (*this)(3, 3) = sin_q_HFE;
    (*this)(3, 4) = cos_q_HFE;
    (*this)(5, 3) = -cos_q_HFE;
    (*this)(5, 4) = sin_q_HFE;
    return *this;
}
ForceTransforms::Type_fr_u1_X_fr_u0::Type_fr_u1_X_fr_u0() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
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
    (*this)(3, 3) = 1.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const ForceTransforms::Type_fr_u1_X_fr_u0& ForceTransforms::Type_fr_u1_X_fr_u0::update(const state_t& q) {
    (*this)(0, 4) = q(BH) + tz_BH;
    (*this)(1, 3) = -q(BH) - tz_BH;
    return *this;
}
ForceTransforms::Type_fr_u0_X_fr_u1::Type_fr_u0_X_fr_u1() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
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
    (*this)(3, 3) = 1.0;
    (*this)(3, 4) = 0.0;
    (*this)(3, 5) = 0.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 0.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 1.0;
}

const ForceTransforms::Type_fr_u0_X_fr_u1& ForceTransforms::Type_fr_u0_X_fr_u1::update(const state_t& q) {
    (*this)(0, 4) = -q(BH) - tz_BH;
    (*this)(1, 3) = q(BH) + tz_BH;
    return *this;
}
ForceTransforms::Type_fr_u2_X_fr_u1::Type_fr_u2_X_fr_u1() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = -1.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(2, 0) = 1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 4) = 0.0;
    (*this)(2, 5) = 0.0;
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
    (*this)(4, 4) = 1.0;
    (*this)(4, 5) = 0.0;
    (*this)(5, 0) = 0.0;
    (*this)(5, 1) = 0.0;
    (*this)(5, 2) = 0.0;
    (*this)(5, 3) = 1.0;
    (*this)(5, 4) = 0.0;
    (*this)(5, 5) = 0.0;
}

const ForceTransforms::Type_fr_u2_X_fr_u1& ForceTransforms::Type_fr_u2_X_fr_u1::update(const state_t& q) {
    (*this)(0, 4) = q(BX);
    (*this)(1, 5) = q(BX);
    return *this;
}
ForceTransforms::Type_fr_u1_X_fr_u2::Type_fr_u1_X_fr_u2() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(2, 5) = 0.0;
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

const ForceTransforms::Type_fr_u1_X_fr_u2& ForceTransforms::Type_fr_u1_X_fr_u2::update(const state_t& q) {
    (*this)(1, 3) = q(BX);
    (*this)(2, 4) = q(BX);
    return *this;
}
ForceTransforms::Type_fr_body_X_fr_u2::Type_fr_body_X_fr_u2() {
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

const ForceTransforms::Type_fr_body_X_fr_u2& ForceTransforms::Type_fr_body_X_fr_u2::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = cos_q_HFE;
    (*this)(0, 2) = sin_q_HFE;
    (*this)(1, 0) = -sin_q_HFE;
    (*this)(1, 2) = cos_q_HFE;
    (*this)(3, 3) = cos_q_HFE;
    (*this)(3, 5) = sin_q_HFE;
    (*this)(4, 3) = -sin_q_HFE;
    (*this)(4, 5) = cos_q_HFE;
    return *this;
}
ForceTransforms::Type_fr_u2_X_fr_body::Type_fr_u2_X_fr_body() {
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

const ForceTransforms::Type_fr_u2_X_fr_body& ForceTransforms::Type_fr_u2_X_fr_body::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = cos_q_HFE;
    (*this)(0, 1) = -sin_q_HFE;
    (*this)(2, 0) = sin_q_HFE;
    (*this)(2, 1) = cos_q_HFE;
    (*this)(3, 3) = cos_q_HFE;
    (*this)(3, 4) = -sin_q_HFE;
    (*this)(5, 3) = sin_q_HFE;
    (*this)(5, 4) = cos_q_HFE;
    return *this;
}
ForceTransforms::Type_fr_leg_X_fr_body::Type_fr_leg_X_fr_body() {
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
    (*this)(2, 4) = -tx_KFE;    // Maxima DSL: -_k__tx_KFE
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

const ForceTransforms::Type_fr_leg_X_fr_body& ForceTransforms::Type_fr_leg_X_fr_body::update(const state_t& q) {
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = cos_q_KFE;
    (*this)(0, 1) = sin_q_KFE;
    (*this)(0, 5) = tx_KFE * sin_q_KFE;
    (*this)(1, 0) = -sin_q_KFE;
    (*this)(1, 1) = cos_q_KFE;
    (*this)(1, 5) = tx_KFE * cos_q_KFE;
    (*this)(3, 3) = cos_q_KFE;
    (*this)(3, 4) = sin_q_KFE;
    (*this)(4, 3) = -sin_q_KFE;
    (*this)(4, 4) = cos_q_KFE;
    return *this;
}
ForceTransforms::Type_fr_body_X_fr_leg::Type_fr_body_X_fr_leg() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(0, 4) = 0.0;
    (*this)(0, 5) = 0.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(1, 4) = 0.0;
    (*this)(1, 5) = -tx_KFE;    // Maxima DSL: -_k__tx_KFE
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

const ForceTransforms::Type_fr_body_X_fr_leg& ForceTransforms::Type_fr_body_X_fr_leg::update(const state_t& q) {
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = cos_q_KFE;
    (*this)(0, 1) = -sin_q_KFE;
    (*this)(1, 0) = sin_q_KFE;
    (*this)(1, 1) = cos_q_KFE;
    (*this)(2, 3) = tx_KFE * sin_q_KFE;
    (*this)(2, 4) = tx_KFE * cos_q_KFE;
    (*this)(3, 3) = cos_q_KFE;
    (*this)(3, 4) = -sin_q_KFE;
    (*this)(4, 3) = sin_q_KFE;
    (*this)(4, 4) = cos_q_KFE;
    return *this;
}

HomogeneousTransforms::Type_fr_u0_X_body::Type_fr_u0_X_body() {
    (*this)(0, 2) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u0_X_body& HomogeneousTransforms::Type_fr_u0_X_body::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = sin_q_HFE;
    (*this)(0, 1) = cos_q_HFE;
    (*this)(0, 3) = q(BX);
    (*this)(2, 0) = -cos_q_HFE;
    (*this)(2, 1) = sin_q_HFE;
    (*this)(2, 3) = q(BH) + tz_BH;
    return *this;
}
HomogeneousTransforms::Type_fr_u0_X_knee::Type_fr_u0_X_knee() {
    (*this)(0, 2) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u0_X_knee& HomogeneousTransforms::Type_fr_u0_X_knee::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(0, 1) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(0, 3) = (tx_KFE * sin_q_HFE) + q(BX);
    (*this)(2, 0) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(2, 1) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(2, 3) = (-tx_KFE * cos_q_HFE) + q(BH) + tz_BH;
    return *this;
}
HomogeneousTransforms::Type_fr_u0_X_foot::Type_fr_u0_X_foot() {
    (*this)(0, 2) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u0_X_foot& HomogeneousTransforms::Type_fr_u0_X_foot::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(0, 1) = (cos_q_HFE * cos_q_KFE) - (sin_q_HFE * sin_q_KFE);
    (*this)(0, 3) =
            (tx_foot * cos_q_HFE * sin_q_KFE) + (tx_foot * sin_q_HFE * cos_q_KFE) + (tx_KFE * sin_q_HFE) + q(BX);
    (*this)(2, 0) = (sin_q_HFE * sin_q_KFE) - (cos_q_HFE * cos_q_KFE);
    (*this)(2, 1) = (cos_q_HFE * sin_q_KFE) + (sin_q_HFE * cos_q_KFE);
    (*this)(2, 3) =
            (tx_foot * sin_q_HFE * sin_q_KFE) - (tx_foot * cos_q_HFE * cos_q_KFE) - (tx_KFE * cos_q_HFE) + q(BH) +
            tz_BH;
    return *this;
}
HomogeneousTransforms::Type_fr_u0_X_fr_BH::Type_fr_u0_X_fr_BH() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(2, 3) = tz_BH;    // Maxima DSL: _k__tz_BH
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u0_X_fr_BH& HomogeneousTransforms::Type_fr_u0_X_fr_BH::update(const state_t& q) {
    return *this;
}
HomogeneousTransforms::Type_fr_u0_X_fr_BX::Type_fr_u0_X_fr_BX() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u0_X_fr_BX& HomogeneousTransforms::Type_fr_u0_X_fr_BX::update(const state_t& q) {
    (*this)(2, 3) = q(BH) + tz_BH;
    return *this;
}
HomogeneousTransforms::Type_fr_u0_X_fr_HFE::Type_fr_u0_X_fr_HFE() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 1.0;
    (*this)(0, 2) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u0_X_fr_HFE& HomogeneousTransforms::Type_fr_u0_X_fr_HFE::update(const state_t& q) {
    (*this)(0, 3) = q(BX);
    (*this)(2, 3) = q(BH) + tz_BH;
    return *this;
}
HomogeneousTransforms::Type_fr_u0_X_fr_KFE::Type_fr_u0_X_fr_KFE() {
    (*this)(0, 2) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u0_X_fr_KFE& HomogeneousTransforms::Type_fr_u0_X_fr_KFE::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = sin_q_HFE;
    (*this)(0, 1) = cos_q_HFE;
    (*this)(0, 3) = (tx_KFE * sin_q_HFE) + q(BX);
    (*this)(2, 0) = -cos_q_HFE;
    (*this)(2, 1) = sin_q_HFE;
    (*this)(2, 3) = (-tx_KFE * cos_q_HFE) + q(BH) + tz_BH;
    return *this;
}
HomogeneousTransforms::Type_fr_u1_X_fr_u0::Type_fr_u1_X_fr_u0() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u1_X_fr_u0& HomogeneousTransforms::Type_fr_u1_X_fr_u0::update(const state_t& q) {
    (*this)(2, 3) = -q(BH) - tz_BH;
    return *this;
}
HomogeneousTransforms::Type_fr_u0_X_fr_u1::Type_fr_u0_X_fr_u1() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 1.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u0_X_fr_u1& HomogeneousTransforms::Type_fr_u0_X_fr_u1::update(const state_t& q) {
    (*this)(2, 3) = q(BH) + tz_BH;
    return *this;
}
HomogeneousTransforms::Type_fr_u2_X_fr_u1::Type_fr_u2_X_fr_u1() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = -1.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = 1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u2_X_fr_u1& HomogeneousTransforms::Type_fr_u2_X_fr_u1::update(const state_t& q) {
    (*this)(2, 3) = -q(BX);
    return *this;
}
HomogeneousTransforms::Type_fr_u1_X_fr_u2::Type_fr_u1_X_fr_u2() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 1.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 1.0;
    (*this)(1, 2) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = -1.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_u1_X_fr_u2& HomogeneousTransforms::Type_fr_u1_X_fr_u2::update(const state_t& q) {
    (*this)(0, 3) = q(BX);
    return *this;
}
HomogeneousTransforms::Type_fr_body_X_fr_u2::Type_fr_body_X_fr_u2() {
    (*this)(0, 1) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 3) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = -1.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 0.0;
    (*this)(3, 2) = 0.0;
    (*this)(3, 3) = 1.0;
}

const HomogeneousTransforms::Type_fr_body_X_fr_u2&
HomogeneousTransforms::Type_fr_body_X_fr_u2::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = cos_q_HFE;
    (*this)(0, 2) = sin_q_HFE;
    (*this)(1, 0) = -sin_q_HFE;
    (*this)(1, 2) = cos_q_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_u2_X_fr_body::Type_fr_u2_X_fr_body() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
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

const HomogeneousTransforms::Type_fr_u2_X_fr_body&
HomogeneousTransforms::Type_fr_u2_X_fr_body::update(const state_t& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    (*this)(0, 0) = cos_q_HFE;
    (*this)(0, 1) = -sin_q_HFE;
    (*this)(2, 0) = sin_q_HFE;
    (*this)(2, 1) = cos_q_HFE;
    return *this;
}
HomogeneousTransforms::Type_fr_leg_X_fr_body::Type_fr_leg_X_fr_body() {
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

const HomogeneousTransforms::Type_fr_leg_X_fr_body&
HomogeneousTransforms::Type_fr_leg_X_fr_body::update(const state_t& q) {
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = cos_q_KFE;
    (*this)(0, 1) = sin_q_KFE;
    (*this)(0, 3) = -tx_KFE * cos_q_KFE;
    (*this)(1, 0) = -sin_q_KFE;
    (*this)(1, 1) = cos_q_KFE;
    (*this)(1, 3) = tx_KFE * sin_q_KFE;
    return *this;
}
HomogeneousTransforms::Type_fr_body_X_fr_leg::Type_fr_body_X_fr_leg() {
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = tx_KFE;    // Maxima DSL: _k__tx_KFE
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

const HomogeneousTransforms::Type_fr_body_X_fr_leg&
HomogeneousTransforms::Type_fr_body_X_fr_leg::update(const state_t& q) {
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(0, 0) = cos_q_KFE;
    (*this)(0, 1) = -sin_q_KFE;
    (*this)(1, 0) = sin_q_KFE;
    (*this)(1, 1) = cos_q_KFE;
    return *this;
}

