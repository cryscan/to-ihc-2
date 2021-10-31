#ifndef RCG_BIPED_MODEL_CONSTANTS_H_
#define RCG_BIPED_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace Biped {
    namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types

        const Scalar ty_L_HAA = 0.10000000149011612;
        const Scalar tz_L_HAA = -0.15000000596046448;
        const Scalar tx_L_HFE = 0.07999999821186066;
        const Scalar tx_L_KFE = 0.3499999940395355;
        const Scalar ty_R_HAA = -0.10000000149011612;
        const Scalar tz_R_HAA = -0.15000000596046448;
        const Scalar tx_R_HFE = 0.07999999821186066;
        const Scalar tx_R_KFE = 0.3499999940395355;
        const Scalar tz_body = 0.15000000596046448;
        const Scalar tx_L_foot = 0.33000001311302185;
        const Scalar tx_R_foot = 0.33000001311302185;
        const Scalar m_trunk = 1.0;
        const Scalar ix_trunk = 0.30000001192092896;
        const Scalar iy_trunk = 0.30000001192092896;
        const Scalar iz_trunk = 0.30000001192092896;
        const Scalar m_L_hip = 0.2930000126361847;
        const Scalar comx_L_hip = 0.04262999817728996;
        const Scalar comz_L_hip = 0.16931000351905823;
        const Scalar ix_L_hip = 0.013470499776303768;
        const Scalar ixy_L_hip = 3.5999998999614036E-6;
        const Scalar ixz_L_hip = 0.002273400081321597;
        const Scalar iy_L_hip = 0.014417099766433239;
        const Scalar iyz_L_hip = 5.100000180391362E-6;
        const Scalar iz_L_hip = 0.0011033000191673636;
        const Scalar m_L_thigh = 0.2637999951839447;
        const Scalar comx_L_thigh = 0.15073999762535095;
        const Scalar comy_L_thigh = -0.026249999180436134;
        const Scalar ix_L_thigh = 5.494999932125211E-4;
        const Scalar ixy_L_thigh = -7.418000022880733E-4;
        const Scalar ixz_L_thigh = -1.0200000360782724E-5;
        const Scalar iy_L_thigh = 0.008713600225746632;
        const Scalar iyz_L_thigh = -2.100000074278796E-6;
        const Scalar iz_L_thigh = 0.00898709986358881;
        const Scalar m_L_shin = 0.08810000121593475;
        const Scalar comx_L_shin = 0.12540000677108765;
        const Scalar comy_L_shin = 5.000000237487257E-4;
        const Scalar comz_L_shin = -9.999999747378752E-5;
        const Scalar ix_L_shin = 4.6799999836366624E-5;
        const Scalar iy_L_shin = 0.002640899969264865;
        const Scalar iz_L_shin = 0.0026181000284850597;
        const Scalar m_R_hip = 0.2930000126361847;
        const Scalar comx_R_hip = 0.04262999817728996;
        const Scalar comz_R_hip = -0.16931000351905823;
        const Scalar ix_R_hip = 0.013470499776303768;
        const Scalar ixy_R_hip = -3.5999998999614036E-6;
        const Scalar ixz_R_hip = -0.002273400081321597;
        const Scalar iy_R_hip = 0.014417099766433239;
        const Scalar iyz_R_hip = 5.100000180391362E-6;
        const Scalar iz_R_hip = 0.0011033000191673636;
        const Scalar m_R_thigh = 0.2637999951839447;
        const Scalar comx_R_thigh = 0.15073999762535095;
        const Scalar comy_R_thigh = -0.026249999180436134;
        const Scalar ix_R_thigh = 5.494999932125211E-4;
        const Scalar ixy_R_thigh = -7.418000022880733E-4;
        const Scalar ixz_R_thigh = -1.0200000360782724E-5;
        const Scalar iy_R_thigh = 0.008713600225746632;
        const Scalar iyz_R_thigh = -2.100000074278796E-6;
        const Scalar iz_R_thigh = 0.00898709986358881;
        const Scalar m_R_shin = 0.08810000121593475;
        const Scalar comx_R_shin = 0.12540000677108765;
        const Scalar comy_R_shin = 5.000000237487257E-4;
        const Scalar comz_R_shin = -9.999999747378752E-5;
        const Scalar ix_R_shin = 4.6799999836366624E-5;
        const Scalar iy_R_shin = 0.002640899969264865;
        const Scalar iz_R_shin = 0.0026181000284850597;

    }
}
#endif
