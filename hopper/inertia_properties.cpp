#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

Hopper::rcg::InertiaProperties::InertiaProperties() {
    com_u1 = Vector3(0.0, 0.0, 0.0);
    tensor_u1.fill(
            0.0,
            com_u1,
            Utils::buildInertiaTensor<Scalar>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    com_u2 = Vector3(0.0, 0.0, 0.0);
    tensor_u2.fill(
            0.0,
            com_u2,
            Utils::buildInertiaTensor<Scalar>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    com_body = Vector3(0.0, 0.0, 0.0);
    tensor_body.fill(
            m_body,
            com_body,
            Utils::buildInertiaTensor<Scalar>(ix_body, iy_body, iz_body, 0.0, 0.0, 0.0));

    com_leg = Vector3(0.0, 0.0, 0.0);
    tensor_leg.fill(
            m_leg,
            com_leg,
            Utils::buildInertiaTensor<Scalar>(ix_leg, iy_leg, iz_leg, 0.0, 0.0, 0.0));

}


void Hopper::rcg::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh) {
    this->params = fresh;
}
