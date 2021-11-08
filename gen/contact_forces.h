//
// Created by cryscan on 11/6/21.
//

#ifndef TO_IHC_2_CONTACT_FORCES_H
#define TO_IHC_2_CONTACT_FORCES_H

#include "../robot_model.h"
#include "ad.h"

namespace gen {
    template<typename T, typename ValueType = double>
    class ContactForces : public ADBase<
            ContactForces<T, ValueType>,
            ModelBase<T>::state_dims + ModelBase<T>::control_dims,
            ModelBase<T>::num_contacts + 1,
            ModelBase<T>::contact_dims,
            ValueType> {
    public:
        using Model = ModelBase<T>;
        using Base = ADBase<
                ContactForces<T, ValueType>,
                Model::state_dims + Model::control_dims,
                Model::num_contacts + 1,
                Model::contact_dims,
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
        explicit ContactForces(const U& u) : Base("contact_forces"), model(u) {}

    private:
        void build_zero() override {
            ScalarVector ad_x = Base::ad_x.template cast<Scalar>();
            ScalarVector ad_y(output_dims);
            CppAD::Independent(ad_x);

            Eigen::DenseIndex it = 0;
            ASSIGN_SEGMENT(model->state, ad_x, it, Model::state_dims)
            ASSIGN_SEGMENT(model->control, ad_x, it, Model::control_dims)
            ASSIGN_SEGMENT(model->d, ad_x, it, Model::num_contacts)
            model->mu = ad_x(it++);

            ad_y << std::get<2>(model->contact()) / model->dt;

            ad_fun[0].template Dependent(ad_y);
            ad_fun[0].optimize("no_compare_op");
        }

        std::shared_ptr<Model> model;
    };

    template<typename T, typename ValueType>
    struct Parameter<ContactForces<T, ValueType>, ValueType> {
        using Model = ModelBase<T>;

        rbd::State<ValueType, Model::joint_state_dims> x;
        Eigen::Matrix<ValueType, Model::control_dims, 1> u;

        Eigen::Matrix<ValueType, Model::num_contacts, 1> d;
        ValueType mu = 1;

        DEF_PARAMETER_FILL(x, u, d, mu)
    };
}

#endif //TO_IHC_2_CONTACT_FORCES_H
