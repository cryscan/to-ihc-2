//
// Created by cryscan on 11/6/21.
//

#ifndef TO_IHC_2_NONLINEAR_TERMS_H
#define TO_IHC_2_NONLINEAR_TERMS_H

#include "../robot_model.h"
#include "ad.h"

namespace gen {
    template<typename T, typename ValueType = double>
    class NonlinearTerms : public ADBase<
            NonlinearTerms<T, ValueType>,
            ModelBase<T>::state_dims,
            0,
            ModelBase<T>::velocity_dims,
            ValueType> {
    public:
        using Model = ModelBase<T>;
        using Base = ADBase<NonlinearTerms<T, ValueType>, Model::state_dims, 0, Model::velocity_dims, ValueType>;

        using Base::input_dims;
        using Base::param_dims;
        using Base::output_dims;

        using Base::ad_fun;
        using typename Base::AD;
        using Scalar = typename Model::Scalar;

        using typename Base::ADVector;
        using ScalarVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

        template<typename U>
        explicit NonlinearTerms(const U& u) : Base("nonlinear_terms"), model(u) {}

    private:
        void build_zero() override {
            ScalarVector ad_x = Base::ad_x.template cast<Scalar>();
            ScalarVector ad_y(output_dims);
            CppAD::Independent(ad_x);

            model->state << ad_x;
            ad_y << model->nonlinear_terms();

            ad_fun[0].template Dependent(ad_y);
            ad_fun[0].optimize("no_compare_op");
        }

        std::shared_ptr<Model> model;
    };

    template<typename T, typename ValueType>
    struct Parameter<NonlinearTerms<T, ValueType>, ValueType> {
        using Model = ModelBase<T>;
        rbd::State<ValueType, Model::joint_state_dims> x;
        DEF_PARAMETER_FILL(x)
    };
}

#endif //TO_IHC_2_NONLINEAR_TERMS_H
