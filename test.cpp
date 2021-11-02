//
// Created by cryscan on 11/2/21.
//

#include <memory>
#include <iostream>

#include "quaternion.h"

#include "biped/rbd_types.h"

#include "rbd_state.h"
#include "robot_model.h"
#include "biped_model.h"

using Biped::rcg::ScalarTraits;

template<typename Scalar>
void set_init_position(rbd::Position<Scalar, Biped::rcg::JointSpaceDimension>& position) {
    typename rbd::Position<Scalar, Biped::rcg::JointSpaceDimension>::Velocity delta;
    delta.base_angular() << 0, M_PI_2, 0;

    position.base_position() << 0, 0, 0.7;
    position += delta;

    position.joint_position() << 0, M_PI_4, -M_PI_2, 0, M_PI_4, -M_PI_2;
}

template<typename T>
void step(ModelBase<T>& model) {
    using Model = ModelBase<T>;

    auto q = model.state.position();
    auto u = model.state.velocity();
    auto dt = model.dt;

    model.state.position() = q + u * dt / 2;
    auto[m_h, m_Jt_p] = model.contact();

    decltype(u) ue = u + m_h * dt + m_Jt_p;
    q += (u + ue) * dt / 2;

    model.state.velocity() = ue;
    model.state.position() = q;
}

void quaternion_forward_zero(double const* in,
                             double* out) {
    //independent variables
    const double* x = in;

    //dependent variables
    double* y = out;

    // auxiliary variables
    double v[4];

    v[0] = 0.5 * sqrt(x[6] * x[6] + x[5] * x[5] + x[4] * x[4]);
    v[1] = cos(v[0]);
    v[0] = sin(v[0]);
    v[2] = v[0] * x[4];
    v[3] = v[0] * x[6];
    v[0] = v[0] * x[5];
    y[0] = x[0] * v[1] + x[3] * v[2] + x[1] * v[3] - x[2] * v[0];
    y[1] = x[1] * v[1] + x[3] * v[0] + x[2] * v[2] - x[0] * v[3];
    y[2] = x[2] * v[1] + x[3] * v[3] + x[0] * v[0] - x[1] * v[2];
    y[3] = x[3] * v[1] - x[0] * v[2] - x[1] * v[0] - x[2] * v[3];
}

int main() {
    QuaternionRotation<double> quaternion("quaternion");
    quaternion.build_map();

    quaternion.params.rotation = Eigen::Quaternion<double>::Identity();
    quaternion.params.velocity << M_PI_2, 0, 0;

    quaternion.evaluate();
    std::cout << quaternion.f.transpose() << std::endl;

    auto& rotation = quaternion.params.rotation;
    auto& velocity = quaternion.params.velocity;
    Eigen::AngleAxis<double> aa(velocity.norm(), velocity.normalized());

    std::cout << (rotation * aa).coeffs().transpose() << std::endl;

    double in[7];
    double out[4];

    Eigen::Map<Eigen::VectorXd>(in, 7) << rotation.coeffs(), velocity;
    quaternion_forward_zero(in, out);
    std::cout << Eigen::Map<Eigen::VectorXd>(out, 4).transpose() << std::endl;

    /*
    auto model = std::make_shared<Biped::Model>();

    set_init_position(model->state.position());

    model->state.velocity().base_angular() << M_PI_2, 0, 0;
    // dynamics.params.x.velocity().base_linear() << 0, 0, 0;
    model->state.velocity().joint_velocity() << 0, 0, -M_PI_2, 0, 0, -M_PI_2;

    std::ofstream os("out.txt");
    os << model->state.transpose() << '\n';

    for (int i = 0; i < 200; ++i) {
        auto e = model->end_effector_positions();
        for (int j = 0, k = 2; j < model->num_contacts; ++j, k += 3) {
            model->d(j) = e(k);
        }

        step(*model);
        os << model->state.transpose() << '\n';
    }
     */
}