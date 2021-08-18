#ifndef RCG_HOPPER_MODEL_CONSTANTS_H_
#define RCG_HOPPER_MODEL_CONSTANTS_H_

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

namespace Hopper {
    namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types

        const Scalar tz_BH = 0.6000000238418579;
        const Scalar tx_KFE = 0.30000001192092896;
        const Scalar tx_foot = 0.30000001192092896;
        const Scalar m_body = 2.4000000953674316;
        const Scalar ix_body = 0.006500000134110451;
        const Scalar iy_body = 0.019999999552965164;
        const Scalar iz_body = 0.02250000089406967;
        const Scalar m_leg = 1.600000023841858;
        const Scalar ix_leg = 1.8000000272877514E-4;
        const Scalar iy_leg = 0.012090000323951244;
        const Scalar iz_leg = 0.012090000323951244;

    }
}
#endif
