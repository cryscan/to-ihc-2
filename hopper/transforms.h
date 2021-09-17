#ifndef HOPPER_TRANSFORMS_H_
#define HOPPER_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace Hopper {
    namespace rcg {

        struct Parameters {
            struct AngleFuncValues {
                AngleFuncValues() {
                    update();
                }

                void update() {
                }
            };

            Params_lengths lengths;
            Params_angles angles;
            AngleFuncValues trig = AngleFuncValues();
        };

// The type of the "vector" with the status of the variables
        typedef JointState state_t;

        template<class M>
        using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

        template<class M>
        using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

        template<class M>
        using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
        class MotionTransforms {
        public:
            class Dummy {
            };

            typedef TransformMotion<Dummy>::MatrixType MatrixType;

            struct Type_fr_u0_X_body : public TransformMotion<Type_fr_u0_X_body> {
                Type_fr_u0_X_body();
                const Type_fr_u0_X_body& update(const state_t&);
            };

            struct Type_fr_u0_X_knee : public TransformMotion<Type_fr_u0_X_knee> {
                Type_fr_u0_X_knee();
                const Type_fr_u0_X_knee& update(const state_t&);
            };

            struct Type_fr_u0_X_foot : public TransformMotion<Type_fr_u0_X_foot> {
                Type_fr_u0_X_foot();
                const Type_fr_u0_X_foot& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_BH : public TransformMotion<Type_fr_u0_X_fr_BH> {
                Type_fr_u0_X_fr_BH();
                const Type_fr_u0_X_fr_BH& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_BX : public TransformMotion<Type_fr_u0_X_fr_BX> {
                Type_fr_u0_X_fr_BX();
                const Type_fr_u0_X_fr_BX& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_HFE : public TransformMotion<Type_fr_u0_X_fr_HFE> {
                Type_fr_u0_X_fr_HFE();
                const Type_fr_u0_X_fr_HFE& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_KFE : public TransformMotion<Type_fr_u0_X_fr_KFE> {
                Type_fr_u0_X_fr_KFE();
                const Type_fr_u0_X_fr_KFE& update(const state_t&);
            };

            struct Type_fr_u1_X_fr_u0 : public TransformMotion<Type_fr_u1_X_fr_u0> {
                Type_fr_u1_X_fr_u0();
                const Type_fr_u1_X_fr_u0& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_u1 : public TransformMotion<Type_fr_u0_X_fr_u1> {
                Type_fr_u0_X_fr_u1();
                const Type_fr_u0_X_fr_u1& update(const state_t&);
            };

            struct Type_fr_u2_X_fr_u1 : public TransformMotion<Type_fr_u2_X_fr_u1> {
                Type_fr_u2_X_fr_u1();
                const Type_fr_u2_X_fr_u1& update(const state_t&);
            };

            struct Type_fr_u1_X_fr_u2 : public TransformMotion<Type_fr_u1_X_fr_u2> {
                Type_fr_u1_X_fr_u2();
                const Type_fr_u1_X_fr_u2& update(const state_t&);
            };

            struct Type_fr_body_X_fr_u2 : public TransformMotion<Type_fr_body_X_fr_u2> {
                Type_fr_body_X_fr_u2();
                const Type_fr_body_X_fr_u2& update(const state_t&);
            };

            struct Type_fr_u2_X_fr_body : public TransformMotion<Type_fr_u2_X_fr_body> {
                Type_fr_u2_X_fr_body();
                const Type_fr_u2_X_fr_body& update(const state_t&);
            };

            struct Type_fr_leg_X_fr_body : public TransformMotion<Type_fr_leg_X_fr_body> {
                Type_fr_leg_X_fr_body();
                const Type_fr_leg_X_fr_body& update(const state_t&);
            };

            struct Type_fr_body_X_fr_leg : public TransformMotion<Type_fr_body_X_fr_leg> {
                Type_fr_body_X_fr_leg();
                const Type_fr_body_X_fr_leg& update(const state_t&);
            };

        public:
            MotionTransforms();
            void updateParams(const Params_lengths&, const Params_angles&);

            Type_fr_u0_X_body fr_u0_X_body;
            Type_fr_u0_X_knee fr_u0_X_knee;
            Type_fr_u0_X_foot fr_u0_X_foot;
            Type_fr_u0_X_fr_BH fr_u0_X_fr_BH;
            Type_fr_u0_X_fr_BX fr_u0_X_fr_BX;
            Type_fr_u0_X_fr_HFE fr_u0_X_fr_HFE;
            Type_fr_u0_X_fr_KFE fr_u0_X_fr_KFE;
            Type_fr_u1_X_fr_u0 fr_u1_X_fr_u0;
            Type_fr_u0_X_fr_u1 fr_u0_X_fr_u1;
            Type_fr_u2_X_fr_u1 fr_u2_X_fr_u1;
            Type_fr_u1_X_fr_u2 fr_u1_X_fr_u2;
            Type_fr_body_X_fr_u2 fr_body_X_fr_u2;
            Type_fr_u2_X_fr_body fr_u2_X_fr_body;
            Type_fr_leg_X_fr_body fr_leg_X_fr_body;
            Type_fr_body_X_fr_leg fr_body_X_fr_leg;

        protected:
            Parameters params;

        }; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
        class ForceTransforms {
        public:
            class Dummy {
            };

            typedef TransformForce<Dummy>::MatrixType MatrixType;

            struct Type_fr_u0_X_body : public TransformForce<Type_fr_u0_X_body> {
                Type_fr_u0_X_body();
                const Type_fr_u0_X_body& update(const state_t&);
            };

            struct Type_fr_u0_X_knee : public TransformForce<Type_fr_u0_X_knee> {
                Type_fr_u0_X_knee();
                const Type_fr_u0_X_knee& update(const state_t&);
            };

            struct Type_fr_u0_X_foot : public TransformForce<Type_fr_u0_X_foot> {
                Type_fr_u0_X_foot();
                const Type_fr_u0_X_foot& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_BH : public TransformForce<Type_fr_u0_X_fr_BH> {
                Type_fr_u0_X_fr_BH();
                const Type_fr_u0_X_fr_BH& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_BX : public TransformForce<Type_fr_u0_X_fr_BX> {
                Type_fr_u0_X_fr_BX();
                const Type_fr_u0_X_fr_BX& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_HFE : public TransformForce<Type_fr_u0_X_fr_HFE> {
                Type_fr_u0_X_fr_HFE();
                const Type_fr_u0_X_fr_HFE& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_KFE : public TransformForce<Type_fr_u0_X_fr_KFE> {
                Type_fr_u0_X_fr_KFE();
                const Type_fr_u0_X_fr_KFE& update(const state_t&);
            };

            struct Type_fr_u1_X_fr_u0 : public TransformForce<Type_fr_u1_X_fr_u0> {
                Type_fr_u1_X_fr_u0();
                const Type_fr_u1_X_fr_u0& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_u1 : public TransformForce<Type_fr_u0_X_fr_u1> {
                Type_fr_u0_X_fr_u1();
                const Type_fr_u0_X_fr_u1& update(const state_t&);
            };

            struct Type_fr_u2_X_fr_u1 : public TransformForce<Type_fr_u2_X_fr_u1> {
                Type_fr_u2_X_fr_u1();
                const Type_fr_u2_X_fr_u1& update(const state_t&);
            };

            struct Type_fr_u1_X_fr_u2 : public TransformForce<Type_fr_u1_X_fr_u2> {
                Type_fr_u1_X_fr_u2();
                const Type_fr_u1_X_fr_u2& update(const state_t&);
            };

            struct Type_fr_body_X_fr_u2 : public TransformForce<Type_fr_body_X_fr_u2> {
                Type_fr_body_X_fr_u2();
                const Type_fr_body_X_fr_u2& update(const state_t&);
            };

            struct Type_fr_u2_X_fr_body : public TransformForce<Type_fr_u2_X_fr_body> {
                Type_fr_u2_X_fr_body();
                const Type_fr_u2_X_fr_body& update(const state_t&);
            };

            struct Type_fr_leg_X_fr_body : public TransformForce<Type_fr_leg_X_fr_body> {
                Type_fr_leg_X_fr_body();
                const Type_fr_leg_X_fr_body& update(const state_t&);
            };

            struct Type_fr_body_X_fr_leg : public TransformForce<Type_fr_body_X_fr_leg> {
                Type_fr_body_X_fr_leg();
                const Type_fr_body_X_fr_leg& update(const state_t&);
            };

        public:
            ForceTransforms();
            void updateParams(const Params_lengths&, const Params_angles&);

            Type_fr_u0_X_body fr_u0_X_body;
            Type_fr_u0_X_knee fr_u0_X_knee;
            Type_fr_u0_X_foot fr_u0_X_foot;
            Type_fr_u0_X_fr_BH fr_u0_X_fr_BH;
            Type_fr_u0_X_fr_BX fr_u0_X_fr_BX;
            Type_fr_u0_X_fr_HFE fr_u0_X_fr_HFE;
            Type_fr_u0_X_fr_KFE fr_u0_X_fr_KFE;
            Type_fr_u1_X_fr_u0 fr_u1_X_fr_u0;
            Type_fr_u0_X_fr_u1 fr_u0_X_fr_u1;
            Type_fr_u2_X_fr_u1 fr_u2_X_fr_u1;
            Type_fr_u1_X_fr_u2 fr_u1_X_fr_u2;
            Type_fr_body_X_fr_u2 fr_body_X_fr_u2;
            Type_fr_u2_X_fr_body fr_u2_X_fr_body;
            Type_fr_leg_X_fr_body fr_leg_X_fr_body;
            Type_fr_body_X_fr_leg fr_body_X_fr_leg;

        protected:
            Parameters params;

        }; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
        class HomogeneousTransforms {
        public:
            class Dummy {
            };

            typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

            struct Type_fr_u0_X_body : public TransformHomogeneous<Type_fr_u0_X_body> {
                Type_fr_u0_X_body();
                const Type_fr_u0_X_body& update(const state_t&);
            };

            struct Type_fr_u0_X_knee : public TransformHomogeneous<Type_fr_u0_X_knee> {
                Type_fr_u0_X_knee();
                const Type_fr_u0_X_knee& update(const state_t&);
            };

            struct Type_fr_u0_X_foot : public TransformHomogeneous<Type_fr_u0_X_foot> {
                Type_fr_u0_X_foot();
                const Type_fr_u0_X_foot& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_BH : public TransformHomogeneous<Type_fr_u0_X_fr_BH> {
                Type_fr_u0_X_fr_BH();
                const Type_fr_u0_X_fr_BH& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_BX : public TransformHomogeneous<Type_fr_u0_X_fr_BX> {
                Type_fr_u0_X_fr_BX();
                const Type_fr_u0_X_fr_BX& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_HFE : public TransformHomogeneous<Type_fr_u0_X_fr_HFE> {
                Type_fr_u0_X_fr_HFE();
                const Type_fr_u0_X_fr_HFE& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_KFE : public TransformHomogeneous<Type_fr_u0_X_fr_KFE> {
                Type_fr_u0_X_fr_KFE();
                const Type_fr_u0_X_fr_KFE& update(const state_t&);
            };

            struct Type_fr_u1_X_fr_u0 : public TransformHomogeneous<Type_fr_u1_X_fr_u0> {
                Type_fr_u1_X_fr_u0();
                const Type_fr_u1_X_fr_u0& update(const state_t&);
            };

            struct Type_fr_u0_X_fr_u1 : public TransformHomogeneous<Type_fr_u0_X_fr_u1> {
                Type_fr_u0_X_fr_u1();
                const Type_fr_u0_X_fr_u1& update(const state_t&);
            };

            struct Type_fr_u2_X_fr_u1 : public TransformHomogeneous<Type_fr_u2_X_fr_u1> {
                Type_fr_u2_X_fr_u1();
                const Type_fr_u2_X_fr_u1& update(const state_t&);
            };

            struct Type_fr_u1_X_fr_u2 : public TransformHomogeneous<Type_fr_u1_X_fr_u2> {
                Type_fr_u1_X_fr_u2();
                const Type_fr_u1_X_fr_u2& update(const state_t&);
            };

            struct Type_fr_body_X_fr_u2 : public TransformHomogeneous<Type_fr_body_X_fr_u2> {
                Type_fr_body_X_fr_u2();
                const Type_fr_body_X_fr_u2& update(const state_t&);
            };

            struct Type_fr_u2_X_fr_body : public TransformHomogeneous<Type_fr_u2_X_fr_body> {
                Type_fr_u2_X_fr_body();
                const Type_fr_u2_X_fr_body& update(const state_t&);
            };

            struct Type_fr_leg_X_fr_body : public TransformHomogeneous<Type_fr_leg_X_fr_body> {
                Type_fr_leg_X_fr_body();
                const Type_fr_leg_X_fr_body& update(const state_t&);
            };

            struct Type_fr_body_X_fr_leg : public TransformHomogeneous<Type_fr_body_X_fr_leg> {
                Type_fr_body_X_fr_leg();
                const Type_fr_body_X_fr_leg& update(const state_t&);
            };

        public:
            HomogeneousTransforms();
            void updateParams(const Params_lengths&, const Params_angles&);

            Type_fr_u0_X_body fr_u0_X_body;
            Type_fr_u0_X_knee fr_u0_X_knee;
            Type_fr_u0_X_foot fr_u0_X_foot;
            Type_fr_u0_X_fr_BH fr_u0_X_fr_BH;
            Type_fr_u0_X_fr_BX fr_u0_X_fr_BX;
            Type_fr_u0_X_fr_HFE fr_u0_X_fr_HFE;
            Type_fr_u0_X_fr_KFE fr_u0_X_fr_KFE;
            Type_fr_u1_X_fr_u0 fr_u1_X_fr_u0;
            Type_fr_u0_X_fr_u1 fr_u0_X_fr_u1;
            Type_fr_u2_X_fr_u1 fr_u2_X_fr_u1;
            Type_fr_u1_X_fr_u2 fr_u1_X_fr_u2;
            Type_fr_body_X_fr_u2 fr_body_X_fr_u2;
            Type_fr_u2_X_fr_body fr_u2_X_fr_body;
            Type_fr_leg_X_fr_body fr_leg_X_fr_body;
            Type_fr_body_X_fr_leg fr_body_X_fr_leg;

        protected:
            Parameters params;

        }; //class 'HomogeneousTransforms'

    }
}

#endif
