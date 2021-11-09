//
// Created by cryscan on 11/9/21.
//

#ifndef TO_IHC_2_REGULATOR_H
#define TO_IHC_2_REGULATOR_H

#include "../robot_model.h"
#include "ad.h"

namespace gen {
    template<typename T, typename ValueType = double>
    class Regulator : public ADBase<
            Regulator<T, ValueType>,
            ModelBase<T>::state_dims + ModelBase<T>::contact_dims,
            3 * ModelBase<T>::joint_state_dims + 3,
            ModelBase<T>::control_dims,
            ValueType> {
    public:
        using Model = ModelBase<T>;
        using Base = ADBase<
                Regulator<T, ValueType>,
                Model::state_dims + Model::contact_dims,
                3 * Model::joint_state_dims + 3,
                Model::control_dims,
                ValueType>;

        using Base::input_dims;
        using Base::param_dims;
        using Base::output_dims;

        using Base::ad_fun;

        using typename Base::AD;
        using Scalar = typename Model::Scalar;

        using typename Base::ADVector;
        using ScalarVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

        template<typename U>
        explicit Regulator(const U& u) : Base("regulator"), model(u) {}

    private:
        void build_zero() override {
            using JointState = typename Model::JointState;
            using ContactVector = typename Model::ContactVector;

            ScalarVector ad_x = Base::ad_x.template cast<Scalar>();
            ScalarVector ad_y(output_dims);
            CppAD::Independent(ad_x);

            ContactVector f;
            JointState qd, vd, ad;
            Scalar kp, kv, ka;

            Eigen::DenseIndex it = 0;
            ASSIGN_SEGMENT(model->state, ad_x, it, Model::state_dims)
            ASSIGN_SEGMENT(f, ad_x, it, Model::contact_dims)
            ASSIGN_SEGMENT(qd, ad_x, it, Model::joint_state_dims)
            ASSIGN_SEGMENT(vd, ad_x, it, Model::joint_state_dims)
            ASSIGN_SEGMENT(ad, ad_x, it, Model::joint_state_dims)
            kp = ad_x(it++);
            kv = ad_x(it++);
            ka = ad_x(it++);

            auto q = model->state.position().joint_position();
            auto u = model->state.velocity().joint_velocity();

            JointState ep = qd - q, ev = vd - u;
            JointState qdd = ad + kv * ev + kp * ep;
            auto[tau, _] = model->id(qdd, f);
            ad_y << tau;

            ad_fun[0].template Dependent(ad_y);
            ad_fun[0].optimize("no_compare_op");
        }

        std::shared_ptr<Model> model;
    };

    template<typename T, typename ValueType>
    struct Parameter<Regulator<T, ValueType>, ValueType> {
        using Model = ModelBase<T>;

        rbd::State<ValueType, Model::joint_state_dims> x;
        Eigen::Matrix<ValueType, Model::contact_dims, 1> f;

        Eigen::Matrix<ValueType, Model::joint_state_dims, 1> qd;
        Eigen::Matrix<ValueType, Model::joint_state_dims, 1> vd;
        Eigen::Matrix<ValueType, Model::joint_state_dims, 1> ad;

        ValueType kp, kv, ka = 1;

        DEF_PARAMETER_FILL(x, f, qd, vd, ad, kp, kv, ka)
    };
}

#endif //TO_IHC_2_REGULATOR_H
