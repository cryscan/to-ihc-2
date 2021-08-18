#include "jacobians.h"

Hopper::rcg::Jacobians::Jacobians()
        : fr_u0_J_foot() {}

void Hopper::rcg::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles) {
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

Hopper::rcg::Jacobians::Type_fr_u0_J_foot::Type_fr_u0_J_foot() {
    (*this)(0, 0) = 0.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(0, 3) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(1, 1) = 0.0;
    (*this)(1, 2) = -1.0;
    (*this)(1, 3) = -1.0;
    (*this)(2, 0) = 0.0;
    (*this)(2, 1) = 0.0;
    (*this)(2, 2) = 0.0;
    (*this)(2, 3) = 0.0;
    (*this)(3, 0) = 0.0;
    (*this)(3, 1) = 1.0;
    (*this)(4, 0) = 0.0;
    (*this)(4, 1) = 0.0;
    (*this)(4, 2) = 0.0;
    (*this)(4, 3) = 0.0;
    (*this)(5, 0) = 1.0;
    (*this)(5, 1) = 0.0;
}

const Hopper::rcg::Jacobians::Type_fr_u0_J_foot&
Hopper::rcg::Jacobians::Type_fr_u0_J_foot::update(const JointState& q) {
    Scalar sin_q_HFE = ScalarTraits::sin(q(HFE));
    Scalar cos_q_HFE = ScalarTraits::cos(q(HFE));
    Scalar sin_q_KFE = ScalarTraits::sin(q(KFE));
    Scalar cos_q_KFE = ScalarTraits::cos(q(KFE));
    (*this)(3, 2) = (-tx_foot * sin_q_HFE * sin_q_KFE) + (tx_foot * cos_q_HFE * cos_q_KFE) + (tx_KFE * cos_q_HFE);
    (*this)(3, 3) = (tx_foot * cos_q_HFE * cos_q_KFE) - (tx_foot * sin_q_HFE * sin_q_KFE);
    (*this)(5, 2) = (tx_foot * cos_q_HFE * sin_q_KFE) + (tx_foot * sin_q_HFE * cos_q_KFE) + (tx_KFE * sin_q_HFE);
    (*this)(5, 3) = (tx_foot * cos_q_HFE * sin_q_KFE) + (tx_foot * sin_q_HFE * cos_q_KFE);
    return *this;
}

