//
// Created by cryscan on 11/6/21.
//

#ifndef TO_IHC_2_INERTIA_H
#define TO_IHC_2_INERTIA_H

#include "../robot_model.h"
#include "ad.h"

namespace gen {
    template<typename T, typename ValueType = double>
    class Inertia : public ADBase<
            Inertia<T, ValueType>,
            ModelBase<T>::state_dims,
            0,
            ModelBase<T>::Inertia::SizeAtCompileTime,
            ValueType> {
    public:
        using Model = ModelBase<T>;
        using Base = ADBase<Inertia<T, ValueType>, Model::state_dims, 0, Model::Inertia::SizeAtCompileTime, ValueType>;

        using Base::input_dims;
        using Base::param_dims;
        using Base::output_dims;

        using Base::ad_fun;
        using typename Base::AD;
        using Scalar = typename Model::Scalar;

        using typename Base::ADVector;
        using ScalarVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

        template<typename U>
        using MatrixType = Eigen::Matrix<U, Model::Inertia::RowsAtCompileTime, Model::Inertia::ColsAtCompileTime>;

        template<typename U>
        explicit Inertia(const U& u) : Base("inertia"), model(u) {}

        MatrixType<ValueType> get() const {
            MatrixType<ValueType> matrix;
            Eigen::Map<Eigen::Matrix<ValueType, Eigen::Dynamic, 1>>(matrix.data(), matrix.size()) << Base::f;
            return matrix;
        }

    private:
        void build_zero() override {
            ScalarVector ad_x = Base::ad_x.template cast<Scalar>();
            ScalarVector ad_y(output_dims);
            CppAD::Independent(ad_x);

            model->state << ad_x;
            auto inertia_matrix = model->inertia_matrix();
            ad_y << Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(inertia_matrix.data(), inertia_matrix.size());

            ad_fun[0].template Dependent(ad_y);
            ad_fun[0].optimize("no_compare_op");
        }

        std::shared_ptr<Model> model;
    };

    template<typename T, typename ValueType>
    struct Parameter<Inertia<T, ValueType>, ValueType> {
        using Model = ModelBase<T>;
        rbd::State<ValueType, Model::joint_state_dims> x;
        DEF_PARAMETER_FILL(x)
    };
}

#endif //TO_IHC_2_INERTIA_H
