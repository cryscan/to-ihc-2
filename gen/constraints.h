//
// Created by cryscan on 11/6/21.
//

#ifndef TO_IHC_2_CONSTRAINTS_H
#define TO_IHC_2_CONSTRAINTS_H

#include "../robot_model.h"
#include "ad.h"

namespace gen {
    template<typename T, typename ValueType = double>
    class Constraints : public ADBase<
            Constraints<T, ValueType>,
            ModelBase<T>::state_dims,
            0,
            ModelBase<T>::ContactJacobian::SizeAtCompileTime,
            ValueType> {
    public:
        using Model = ModelBase<T>;
        using Base = ADBase<
                Constraints<T, ValueType>,
                Model::state_dims,
                0,
                Model::ContactJacobian::SizeAtCompileTime,
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
        using MatrixType = Eigen::Matrix<U, Model::Inertia::RowsAtCompileTime, Model::Inertia::ColsAtCompileTime>;

        template<typename U>
        explicit Constraints(const U& u) : Base("constraints"), model(u) {}

        MatrixType<ValueType> get() const {
            return Eigen::Map<MatrixType<ValueType>>(Base::f);
        }

    private:
        void build_zero() override {
            ScalarVector ad_x = Base::ad_x.template cast<Scalar>();
            ScalarVector ad_y(output_dims);
            CppAD::Independent(ad_x);

            model->state << ad_x;
            auto contact_jacobian = model->contact_jacobian();
            ad_y << Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(contact_jacobian.data(),
                                                                         contact_jacobian.size());

            ad_fun[0].template Dependent(ad_y);
            ad_fun[0].optimize("no_compare_op");
        }

        std::shared_ptr<Model> model;
    };

    template<typename T, typename ValueType>
    struct Parameter<Constraints<T, ValueType>, ValueType> {
        using Model = ModelBase<T>;
        rbd::State<ValueType, Model::joint_state_dims> x;
        DEF_PARAMETER_FILL(x)
    };
}

#endif //TO_IHC_2_CONSTRAINTS_H
