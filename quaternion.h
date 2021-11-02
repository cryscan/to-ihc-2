//
// Created by cryscan on 11/2/21.
//

#ifndef TO_IHC_2_QUATERNION_H
#define TO_IHC_2_QUATERNION_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ad.h"

template<typename ValueType>
class QuaternionRotation;

template<typename ValueType>
struct Parameter<QuaternionRotation<ValueType>, ValueType> {
    Eigen::Quaternion<ValueType> rotation;
    Eigen::Matrix<ValueType, 3, 1> velocity;

    template<typename Vector>
    void fill(Eigen::MatrixBase<Vector>& vector) {
        vector << rotation.coeffs(), velocity;
    }
};

template<typename ValueType = double>
class QuaternionRotation : public ADBase<QuaternionRotation<ValueType>, 7, 0, 4, ValueType> {
public:
    using Base = ADBase<QuaternionRotation<ValueType>, 7, 0, 4, ValueType>;
    using Base::input_dims;
    using Base::output_dims;

    using typename Base::AD;
    using typename Base::ADVector;

    using Base::ad_x;
    using Base::ad_fun;

    explicit QuaternionRotation(const std::string& name) : Base(name) {}

private:
    void build_zero() override {
        ADVector ad_y(output_dims);
        CppAD::Independent(ad_x);

        Eigen::Quaternion<AD> a(ad_x.template head<4>());

        Eigen::Matrix<AD, 3, 1> omega = ad_x.template tail<3>();
        AD angle = omega.norm();
        decltype(omega) axis = omega / angle;
        Eigen::AngleAxis<AD> b(angle, axis);

        Eigen::Quaternion<AD> r = a * b;
        ad_y << r.coeffs();

        ad_fun[0].template Dependent(ad_y);
        ad_fun[0].optimize("no_compare_op");
    }
};

#endif //TO_IHC_2_QUATERNION_H
