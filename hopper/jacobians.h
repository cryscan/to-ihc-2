#ifndef HOPPER_JACOBIANS_H_
#define HOPPER_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "transforms.h" // to use the same 'Parameters' struct defined there
#include "model_constants.h"

namespace Hopper {
namespace rcg {

    template<int COLS, class M>
    class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M> {
    };

/**
 *
 */
    class Jacobians {
    public:

        struct Type_fr_u0_J_body : public JacobianT<4, Type_fr_u0_J_body> {
            Type_fr_u0_J_body();
            const Type_fr_u0_J_body& update(const JointState&);
        };


        struct Type_fr_u0_J_knee : public JacobianT<4, Type_fr_u0_J_knee> {
            Type_fr_u0_J_knee();
            const Type_fr_u0_J_knee& update(const JointState&);
        };


        struct Type_fr_u0_J_foot : public JacobianT<4, Type_fr_u0_J_foot> {
            Type_fr_u0_J_foot();
            const Type_fr_u0_J_foot& update(const JointState&);
        };

    public:
        Jacobians();
        void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
    public:
        Type_fr_u0_J_body fr_u0_J_body;
        Type_fr_u0_J_knee fr_u0_J_knee;
        Type_fr_u0_J_foot fr_u0_J_foot;

    protected:
        Parameters params;

    };


}
}

#endif
