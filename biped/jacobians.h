#ifndef BIPED_JACOBIANS_H_
#define BIPED_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "transforms.h" // to use the same 'Parameters' struct defined there
#include "model_constants.h"

namespace Biped {
    namespace rcg {

        template<int COLS, class M>
        class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M> {
        };

/**
 *
 */
        class Jacobians {
        public:

            struct Type_fr_trunk_J_L_foot : public JacobianT<3, Type_fr_trunk_J_L_foot> {
                Type_fr_trunk_J_L_foot();
                const Type_fr_trunk_J_L_foot& update(const JointState&);
            };


            struct Type_fr_trunk_J_R_foot : public JacobianT<3, Type_fr_trunk_J_R_foot> {
                Type_fr_trunk_J_R_foot();
                const Type_fr_trunk_J_R_foot& update(const JointState&);
            };

        public:
            Jacobians();
            void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
        public:
            Type_fr_trunk_J_L_foot fr_trunk_J_L_foot;
            Type_fr_trunk_J_R_foot fr_trunk_J_R_foot;

        protected:
            Parameters params;

        };


    }
}

#endif
