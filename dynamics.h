//
// Created by cryscan on 10/31/21.
//

#ifndef TO_IHC_2_DYNAMICS_H
#define TO_IHC_2_DYNAMICS_H

#include "robot_model.h"
#include "ad.h"

template<typename T, typename ValueType = double>
class Dynamics : public ADBase<
        Dynamics<T, ValueType>,
        ModelBase<T>::state_dims + ModelBase<T>::control_dims,
        ModelBase<T>::num_contacts + 1,
        ModelBase<T>::state_dims,
        ValueType,
        true> {
public:
    using Model = ModelBase<T>;
    using Base = ADBase<
            Dynamics<T, ValueType>,
            Model::state_dims + Model::control_dims,
            Model::num_contacts + 1,
            Model::state_dims,
            ValueType,
            true>;

    using Base::input_dims;
    using Base::param_dims;
    using Base::output_dims;

    using Base::ad_fun;

    using typename Base::AD;
    using Scalar = typename Model::Scalar;

    using typename Base::ADVector;
    using ScalarVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

    template<typename U>
    explicit Dynamics(const U& u) : Base("dynamics"), model(u) {}

private:
    void step() {
        auto q = model->state.position();
        auto u = model->state.velocity();
        auto dt = model->dt;

        model->state.position() = q + u * dt / 2;
        auto[m_h, m_Jt_p, _] = model->contact();

        decltype(u) ue = u + m_h * dt + m_Jt_p;
        q += (u + ue) * dt / 2;

        model->state.velocity() = ue;
        model->state.position() = q;
    }

    void build_zero() override {
        ScalarVector ad_x = Base::ad_x.template cast<Scalar>();
        ScalarVector ad_y(output_dims);
        CppAD::Independent(ad_x);

        Eigen::DenseIndex it = 0;
        ASSIGN_SEGMENT(model->state, ad_x, it, Model::state_dims)
        ASSIGN_SEGMENT(model->control, ad_x, it, Model::control_dims)
        ASSIGN_SEGMENT(model->d, ad_x, it, Model::num_contacts)
        model->mu = ad_x(it++);

        step();
        ad_y << model->state;

        ad_fun[0].template Dependent(ad_y);
        ad_fun[0].optimize("no_compare_op");
    }

    std::shared_ptr<Model> model;
};

template<typename T, typename ValueType>
struct Parameter<Dynamics<T, ValueType>, ValueType> {
    using Model = ModelBase<T>;

    // inputs
    rbd::State<ValueType, Model::joint_state_dims> x;
    Eigen::Matrix<ValueType, Model::control_dims, 1> u;

    // parameters
    Eigen::Matrix<ValueType, Model::num_contacts, 1> d;
    ValueType mu = 1;

    DEF_PARAMETER_FILL(x, u, d, mu)
};

#endif //TO_IHC_2_DYNAMICS_H
