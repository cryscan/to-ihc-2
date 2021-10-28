#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

Biped::rcg::InertiaProperties::InertiaProperties() {
    com_trunk = Vector3(0.0, 0.0, 0.0);
    tensor_trunk.fill(
            m_trunk,
            com_trunk,
            Utils::buildInertiaTensor<Scalar>(ix_trunk, iy_trunk, iz_trunk, 0.0, 0.0, 0.0));

    com_L_hip = Vector3(comx_L_hip, 0.0, comz_L_hip);
    tensor_L_hip.fill(
            m_L_hip,
            com_L_hip,
            Utils::buildInertiaTensor<Scalar>(ix_L_hip, iy_L_hip, iz_L_hip, ixy_L_hip, ixz_L_hip, iyz_L_hip));

    com_L_thigh = Vector3(comx_L_thigh, comy_L_thigh, 0.0);
    tensor_L_thigh.fill(
            m_L_thigh,
            com_L_thigh,
            Utils::buildInertiaTensor<Scalar>(ix_L_thigh, iy_L_thigh, iz_L_thigh, ixy_L_thigh, ixz_L_thigh,
                                              iyz_L_thigh));

    com_L_shin = Vector3(comx_L_shin, comy_L_shin, comz_L_shin);
    tensor_L_shin.fill(
            m_L_shin,
            com_L_shin,
            Utils::buildInertiaTensor<Scalar>(ix_L_shin, iy_L_shin, iz_L_shin, 0.0, 0.0, 0.0));

    com_R_hip = Vector3(comx_R_hip, 0.0, comz_R_hip);
    tensor_R_hip.fill(
            m_R_hip,
            com_R_hip,
            Utils::buildInertiaTensor<Scalar>(ix_R_hip, iy_R_hip, iz_R_hip, ixy_R_hip, ixz_R_hip, iyz_R_hip));

    com_R_thigh = Vector3(comx_R_thigh, comy_R_thigh, 0.0);
    tensor_R_thigh.fill(
            m_R_thigh,
            com_R_thigh,
            Utils::buildInertiaTensor<Scalar>(ix_R_thigh, iy_R_thigh, iz_R_thigh, ixy_R_thigh, ixz_R_thigh,
                                              iyz_R_thigh));

    com_R_shin = Vector3(comx_R_shin, comy_R_shin, comz_R_shin);
    tensor_R_shin.fill(
            m_R_shin,
            com_R_shin,
            Utils::buildInertiaTensor<Scalar>(ix_R_shin, iy_R_shin, iz_R_shin, 0.0, 0.0, 0.0));

}


void Biped::rcg::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh) {
    this->params = fresh;
}
