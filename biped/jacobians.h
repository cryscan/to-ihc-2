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

            struct Type_fr_trunk_J_body : public JacobianT<0, Type_fr_trunk_J_body> {
                Type_fr_trunk_J_body();
                const Type_fr_trunk_J_body& update(const JointState&);
            };


            struct Type_fr_trunk_J_fr_L_hip : public JacobianT<1, Type_fr_trunk_J_fr_L_hip> {
                Type_fr_trunk_J_fr_L_hip();
                const Type_fr_trunk_J_fr_L_hip& update(const JointState&);
            };


            struct Type_fr_trunk_J_fr_R_hip : public JacobianT<1, Type_fr_trunk_J_fr_R_hip> {
                Type_fr_trunk_J_fr_R_hip();
                const Type_fr_trunk_J_fr_R_hip& update(const JointState&);
            };


            struct Type_fr_trunk_J_fr_L_thigh : public JacobianT<2, Type_fr_trunk_J_fr_L_thigh> {
                Type_fr_trunk_J_fr_L_thigh();
                const Type_fr_trunk_J_fr_L_thigh& update(const JointState&);
            };


            struct Type_fr_trunk_J_fr_R_thigh : public JacobianT<2, Type_fr_trunk_J_fr_R_thigh> {
                Type_fr_trunk_J_fr_R_thigh();
                const Type_fr_trunk_J_fr_R_thigh& update(const JointState&);
            };


            struct Type_fr_trunk_J_fr_L_shin : public JacobianT<3, Type_fr_trunk_J_fr_L_shin> {
                Type_fr_trunk_J_fr_L_shin();
                const Type_fr_trunk_J_fr_L_shin& update(const JointState&);
            };


            struct Type_fr_trunk_J_fr_R_shin : public JacobianT<3, Type_fr_trunk_J_fr_R_shin> {
                Type_fr_trunk_J_fr_R_shin();
                const Type_fr_trunk_J_fr_R_shin& update(const JointState&);
            };


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
            Type_fr_trunk_J_body fr_trunk_J_body;
            Type_fr_trunk_J_fr_L_hip fr_trunk_J_fr_L_hip;
            Type_fr_trunk_J_fr_R_hip fr_trunk_J_fr_R_hip;
            Type_fr_trunk_J_fr_L_thigh fr_trunk_J_fr_L_thigh;
            Type_fr_trunk_J_fr_R_thigh fr_trunk_J_fr_R_thigh;
            Type_fr_trunk_J_fr_L_shin fr_trunk_J_fr_L_shin;
            Type_fr_trunk_J_fr_R_shin fr_trunk_J_fr_R_shin;
            Type_fr_trunk_J_L_foot fr_trunk_J_L_foot;
            Type_fr_trunk_J_R_foot fr_trunk_J_R_foot;

        protected:
            Parameters params;

        };


    }
}

#endif
