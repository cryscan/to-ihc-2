#include "jacobians.h"

Biped::rcg::Jacobians::Jacobians()
        : fr_trunk_J_L_foot(),
          fr_trunk_J_R_foot() {}

void Biped::rcg::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles) {
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

Biped::rcg::Jacobians::Type_fr_trunk_J_L_foot::Type_fr_trunk_J_L_foot() {
    (*this)(0, 0) = -1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(3, 0) = 0.0;
}

const Biped::rcg::Jacobians::Type_fr_trunk_J_L_foot&
Biped::rcg::Jacobians::Type_fr_trunk_J_L_foot::update(const JointState& q) {
    Scalar sin_q_L_HAA = ScalarTraits::sin(q(L_HAA));
    Scalar cos_q_L_HAA = ScalarTraits::cos(q(L_HAA));
    Scalar sin_q_L_HFE = ScalarTraits::sin(q(L_HFE));
    Scalar cos_q_L_HFE = ScalarTraits::cos(q(L_HFE));
    Scalar sin_q_L_KFE = ScalarTraits::sin(q(L_KFE));
    Scalar cos_q_L_KFE = ScalarTraits::cos(q(L_KFE));
    (*this)(1, 1) = cos_q_L_HAA;
    (*this)(1, 2) = cos_q_L_HAA;
    (*this)(2, 1) = -sin_q_L_HAA;
    (*this)(2, 2) = -sin_q_L_HAA;
    (*this)(3, 1) = (tx_L_foot * sin_q_L_HFE * sin_q_L_KFE) - (tx_L_foot * cos_q_L_HFE * cos_q_L_KFE) -
                    (tx_L_KFE * cos_q_L_HFE);
    (*this)(3, 2) = (tx_L_foot * sin_q_L_HFE * sin_q_L_KFE) - (tx_L_foot * cos_q_L_HFE * cos_q_L_KFE);
    (*this)(4, 0) = (tx_L_foot * cos_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE) -
                    (tx_L_foot * cos_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) - (tx_L_KFE * cos_q_L_HAA * cos_q_L_HFE) -
                    (tx_L_HFE * cos_q_L_HAA);
    (*this)(4, 1) = (tx_L_foot * sin_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) +
                    (tx_L_foot * sin_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE) + (tx_L_KFE * sin_q_L_HAA * sin_q_L_HFE);
    (*this)(4, 2) = (tx_L_foot * sin_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) +
                    (tx_L_foot * sin_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    (*this)(5, 0) = (-tx_L_foot * sin_q_L_HAA * sin_q_L_HFE * sin_q_L_KFE) +
                    (tx_L_foot * sin_q_L_HAA * cos_q_L_HFE * cos_q_L_KFE) + (tx_L_KFE * sin_q_L_HAA * cos_q_L_HFE) +
                    (tx_L_HFE * sin_q_L_HAA);
    (*this)(5, 1) = (tx_L_foot * cos_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) +
                    (tx_L_foot * cos_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE) + (tx_L_KFE * cos_q_L_HAA * sin_q_L_HFE);
    (*this)(5, 2) = (tx_L_foot * cos_q_L_HAA * cos_q_L_HFE * sin_q_L_KFE) +
                    (tx_L_foot * cos_q_L_HAA * sin_q_L_HFE * cos_q_L_KFE);
    return *this;
}

Biped::rcg::Jacobians::Type_fr_trunk_J_R_foot::Type_fr_trunk_J_R_foot() {
    (*this)(0, 0) = 1.0;
    (*this)(0, 1) = 0.0;
    (*this)(0, 2) = 0.0;
    (*this)(1, 0) = 0.0;
    (*this)(2, 0) = 0.0;
    (*this)(3, 0) = 0.0;
}

const Biped::rcg::Jacobians::Type_fr_trunk_J_R_foot&
Biped::rcg::Jacobians::Type_fr_trunk_J_R_foot::update(const JointState& q) {
    Scalar sin_q_R_HAA = ScalarTraits::sin(q(R_HAA));
    Scalar cos_q_R_HAA = ScalarTraits::cos(q(R_HAA));
    Scalar sin_q_R_HFE = ScalarTraits::sin(q(R_HFE));
    Scalar cos_q_R_HFE = ScalarTraits::cos(q(R_HFE));
    Scalar sin_q_R_KFE = ScalarTraits::sin(q(R_KFE));
    Scalar cos_q_R_KFE = ScalarTraits::cos(q(R_KFE));
    (*this)(1, 1) = cos_q_R_HAA;
    (*this)(1, 2) = cos_q_R_HAA;
    (*this)(2, 1) = sin_q_R_HAA;
    (*this)(2, 2) = sin_q_R_HAA;
    (*this)(3, 1) = (tx_R_foot * sin_q_R_HFE * sin_q_R_KFE) - (tx_R_foot * cos_q_R_HFE * cos_q_R_KFE) -
                    (tx_R_KFE * cos_q_R_HFE);
    (*this)(3, 2) = (tx_R_foot * sin_q_R_HFE * sin_q_R_KFE) - (tx_R_foot * cos_q_R_HFE * cos_q_R_KFE);
    (*this)(4, 0) = (-tx_R_foot * cos_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE) +
                    (tx_R_foot * cos_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE) + (tx_R_KFE * cos_q_R_HAA * cos_q_R_HFE) +
                    (tx_R_HFE * cos_q_R_HAA);
    (*this)(4, 1) = (-tx_R_foot * sin_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) -
                    (tx_R_foot * sin_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE) - (tx_R_KFE * sin_q_R_HAA * sin_q_R_HFE);
    (*this)(4, 2) = (-tx_R_foot * sin_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) -
                    (tx_R_foot * sin_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    (*this)(5, 0) = (-tx_R_foot * sin_q_R_HAA * sin_q_R_HFE * sin_q_R_KFE) +
                    (tx_R_foot * sin_q_R_HAA * cos_q_R_HFE * cos_q_R_KFE) + (tx_R_KFE * sin_q_R_HAA * cos_q_R_HFE) +
                    (tx_R_HFE * sin_q_R_HAA);
    (*this)(5, 1) = (tx_R_foot * cos_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) +
                    (tx_R_foot * cos_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE) + (tx_R_KFE * cos_q_R_HAA * sin_q_R_HFE);
    (*this)(5, 2) = (tx_R_foot * cos_q_R_HAA * cos_q_R_HFE * sin_q_R_KFE) +
                    (tx_R_foot * cos_q_R_HAA * sin_q_R_HFE * cos_q_R_KFE);
    return *this;
}

