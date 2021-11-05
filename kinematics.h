//
// Created by cryscan on 11/1/21.
//

#ifndef TO_IHC_2_KINEMATICS_H
#define TO_IHC_2_KINEMATICS_H

#include "robot_model.h"
#include "ad.h"

template<typename T, typename ValueType = double>
class Kinematics : public ADBase<
        Kinematics<T, ValueType>,
        ModelBase<T>::position_dims,
        0,
        ModelBase<T>::contact_dims,
        ValueType> {
public:
    using Model = ModelBase<T>;
    using Base = ADBase<Kinematics<T, ValueType>, Model::position_dims, 0, Model::contact_dims, ValueType>;

    using Base::input_dims;
    using Base::param_dims;
    using Base::output_dims;

    using Base::ad_fun;
    using typename Base::AD;
    using Scalar = typename Model::Scalar;

    using typename Base::ADVector;
    using ScalarVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    template<typename U>
    explicit Kinematics(const U& u) : Base("kinematics"), model(u) {}

private:
    void build_zero() override {
        ScalarVector ad_x = Base::ad_x.template cast<Scalar>();
        ScalarVector ad_y(output_dims);
        CppAD::Independent(ad_x);

        model->state.position() << ad_x;
        ad_y << model->end_effector_positions();

        ad_fun[0].template Dependent(ad_y);
        ad_fun[0].optimize("no_compare_op");
    }

    std::shared_ptr<Model> model;
};

template<typename T, typename ValueType>
struct Parameter<Kinematics<T, ValueType>, ValueType> {
    rbd::Position<ValueType, ModelBase<T>::joint_state_dims> q;

    DEF_PARAMETER_FILL(q)
};

#endif //TO_IHC_2_KINEMATICS_H
