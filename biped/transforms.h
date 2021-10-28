#ifndef BIPED_TRANSFORMS_H_
#define BIPED_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace Biped {
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

            struct Type_fr_trunk_X_L_foot : public TransformMotion<Type_fr_trunk_X_L_foot> {
                Type_fr_trunk_X_L_foot();
                const Type_fr_trunk_X_L_foot& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_HAA : public TransformMotion<Type_fr_trunk_X_fr_L_HAA> {
                Type_fr_trunk_X_fr_L_HAA();
                const Type_fr_trunk_X_fr_L_HAA& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_HFE : public TransformMotion<Type_fr_trunk_X_fr_L_HFE> {
                Type_fr_trunk_X_fr_L_HFE();
                const Type_fr_trunk_X_fr_L_HFE& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_KFE : public TransformMotion<Type_fr_trunk_X_fr_L_KFE> {
                Type_fr_trunk_X_fr_L_KFE();
                const Type_fr_trunk_X_fr_L_KFE& update(const state_t&);
            };

            struct Type_fr_trunk_X_R_foot : public TransformMotion<Type_fr_trunk_X_R_foot> {
                Type_fr_trunk_X_R_foot();
                const Type_fr_trunk_X_R_foot& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_HAA : public TransformMotion<Type_fr_trunk_X_fr_R_HAA> {
                Type_fr_trunk_X_fr_R_HAA();
                const Type_fr_trunk_X_fr_R_HAA& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_HFE : public TransformMotion<Type_fr_trunk_X_fr_R_HFE> {
                Type_fr_trunk_X_fr_R_HFE();
                const Type_fr_trunk_X_fr_R_HFE& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_KFE : public TransformMotion<Type_fr_trunk_X_fr_R_KFE> {
                Type_fr_trunk_X_fr_R_KFE();
                const Type_fr_trunk_X_fr_R_KFE& update(const state_t&);
            };

            struct Type_fr_L_hip_X_fr_trunk : public TransformMotion<Type_fr_L_hip_X_fr_trunk> {
                Type_fr_L_hip_X_fr_trunk();
                const Type_fr_L_hip_X_fr_trunk& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_hip : public TransformMotion<Type_fr_trunk_X_fr_L_hip> {
                Type_fr_trunk_X_fr_L_hip();
                const Type_fr_trunk_X_fr_L_hip& update(const state_t&);
            };

            struct Type_fr_L_thigh_X_fr_L_hip : public TransformMotion<Type_fr_L_thigh_X_fr_L_hip> {
                Type_fr_L_thigh_X_fr_L_hip();
                const Type_fr_L_thigh_X_fr_L_hip& update(const state_t&);
            };

            struct Type_fr_L_hip_X_fr_L_thigh : public TransformMotion<Type_fr_L_hip_X_fr_L_thigh> {
                Type_fr_L_hip_X_fr_L_thigh();
                const Type_fr_L_hip_X_fr_L_thigh& update(const state_t&);
            };

            struct Type_fr_L_shin_X_fr_L_thigh : public TransformMotion<Type_fr_L_shin_X_fr_L_thigh> {
                Type_fr_L_shin_X_fr_L_thigh();
                const Type_fr_L_shin_X_fr_L_thigh& update(const state_t&);
            };

            struct Type_fr_L_thigh_X_fr_L_shin : public TransformMotion<Type_fr_L_thigh_X_fr_L_shin> {
                Type_fr_L_thigh_X_fr_L_shin();
                const Type_fr_L_thigh_X_fr_L_shin& update(const state_t&);
            };

            struct Type_fr_R_hip_X_fr_trunk : public TransformMotion<Type_fr_R_hip_X_fr_trunk> {
                Type_fr_R_hip_X_fr_trunk();
                const Type_fr_R_hip_X_fr_trunk& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_hip : public TransformMotion<Type_fr_trunk_X_fr_R_hip> {
                Type_fr_trunk_X_fr_R_hip();
                const Type_fr_trunk_X_fr_R_hip& update(const state_t&);
            };

            struct Type_fr_R_thigh_X_fr_R_hip : public TransformMotion<Type_fr_R_thigh_X_fr_R_hip> {
                Type_fr_R_thigh_X_fr_R_hip();
                const Type_fr_R_thigh_X_fr_R_hip& update(const state_t&);
            };

            struct Type_fr_R_hip_X_fr_R_thigh : public TransformMotion<Type_fr_R_hip_X_fr_R_thigh> {
                Type_fr_R_hip_X_fr_R_thigh();
                const Type_fr_R_hip_X_fr_R_thigh& update(const state_t&);
            };

            struct Type_fr_R_shin_X_fr_R_thigh : public TransformMotion<Type_fr_R_shin_X_fr_R_thigh> {
                Type_fr_R_shin_X_fr_R_thigh();
                const Type_fr_R_shin_X_fr_R_thigh& update(const state_t&);
            };

            struct Type_fr_R_thigh_X_fr_R_shin : public TransformMotion<Type_fr_R_thigh_X_fr_R_shin> {
                Type_fr_R_thigh_X_fr_R_shin();
                const Type_fr_R_thigh_X_fr_R_shin& update(const state_t&);
            };

        public:
            MotionTransforms();
            void updateParams(const Params_lengths&, const Params_angles&);

            Type_fr_trunk_X_L_foot fr_trunk_X_L_foot;
            Type_fr_trunk_X_fr_L_HAA fr_trunk_X_fr_L_HAA;
            Type_fr_trunk_X_fr_L_HFE fr_trunk_X_fr_L_HFE;
            Type_fr_trunk_X_fr_L_KFE fr_trunk_X_fr_L_KFE;
            Type_fr_trunk_X_R_foot fr_trunk_X_R_foot;
            Type_fr_trunk_X_fr_R_HAA fr_trunk_X_fr_R_HAA;
            Type_fr_trunk_X_fr_R_HFE fr_trunk_X_fr_R_HFE;
            Type_fr_trunk_X_fr_R_KFE fr_trunk_X_fr_R_KFE;
            Type_fr_L_hip_X_fr_trunk fr_L_hip_X_fr_trunk;
            Type_fr_trunk_X_fr_L_hip fr_trunk_X_fr_L_hip;
            Type_fr_L_thigh_X_fr_L_hip fr_L_thigh_X_fr_L_hip;
            Type_fr_L_hip_X_fr_L_thigh fr_L_hip_X_fr_L_thigh;
            Type_fr_L_shin_X_fr_L_thigh fr_L_shin_X_fr_L_thigh;
            Type_fr_L_thigh_X_fr_L_shin fr_L_thigh_X_fr_L_shin;
            Type_fr_R_hip_X_fr_trunk fr_R_hip_X_fr_trunk;
            Type_fr_trunk_X_fr_R_hip fr_trunk_X_fr_R_hip;
            Type_fr_R_thigh_X_fr_R_hip fr_R_thigh_X_fr_R_hip;
            Type_fr_R_hip_X_fr_R_thigh fr_R_hip_X_fr_R_thigh;
            Type_fr_R_shin_X_fr_R_thigh fr_R_shin_X_fr_R_thigh;
            Type_fr_R_thigh_X_fr_R_shin fr_R_thigh_X_fr_R_shin;

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

            struct Type_fr_trunk_X_L_foot : public TransformForce<Type_fr_trunk_X_L_foot> {
                Type_fr_trunk_X_L_foot();
                const Type_fr_trunk_X_L_foot& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_HAA : public TransformForce<Type_fr_trunk_X_fr_L_HAA> {
                Type_fr_trunk_X_fr_L_HAA();
                const Type_fr_trunk_X_fr_L_HAA& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_HFE : public TransformForce<Type_fr_trunk_X_fr_L_HFE> {
                Type_fr_trunk_X_fr_L_HFE();
                const Type_fr_trunk_X_fr_L_HFE& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_KFE : public TransformForce<Type_fr_trunk_X_fr_L_KFE> {
                Type_fr_trunk_X_fr_L_KFE();
                const Type_fr_trunk_X_fr_L_KFE& update(const state_t&);
            };

            struct Type_fr_trunk_X_R_foot : public TransformForce<Type_fr_trunk_X_R_foot> {
                Type_fr_trunk_X_R_foot();
                const Type_fr_trunk_X_R_foot& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_HAA : public TransformForce<Type_fr_trunk_X_fr_R_HAA> {
                Type_fr_trunk_X_fr_R_HAA();
                const Type_fr_trunk_X_fr_R_HAA& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_HFE : public TransformForce<Type_fr_trunk_X_fr_R_HFE> {
                Type_fr_trunk_X_fr_R_HFE();
                const Type_fr_trunk_X_fr_R_HFE& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_KFE : public TransformForce<Type_fr_trunk_X_fr_R_KFE> {
                Type_fr_trunk_X_fr_R_KFE();
                const Type_fr_trunk_X_fr_R_KFE& update(const state_t&);
            };

            struct Type_fr_L_hip_X_fr_trunk : public TransformForce<Type_fr_L_hip_X_fr_trunk> {
                Type_fr_L_hip_X_fr_trunk();
                const Type_fr_L_hip_X_fr_trunk& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_hip : public TransformForce<Type_fr_trunk_X_fr_L_hip> {
                Type_fr_trunk_X_fr_L_hip();
                const Type_fr_trunk_X_fr_L_hip& update(const state_t&);
            };

            struct Type_fr_L_thigh_X_fr_L_hip : public TransformForce<Type_fr_L_thigh_X_fr_L_hip> {
                Type_fr_L_thigh_X_fr_L_hip();
                const Type_fr_L_thigh_X_fr_L_hip& update(const state_t&);
            };

            struct Type_fr_L_hip_X_fr_L_thigh : public TransformForce<Type_fr_L_hip_X_fr_L_thigh> {
                Type_fr_L_hip_X_fr_L_thigh();
                const Type_fr_L_hip_X_fr_L_thigh& update(const state_t&);
            };

            struct Type_fr_L_shin_X_fr_L_thigh : public TransformForce<Type_fr_L_shin_X_fr_L_thigh> {
                Type_fr_L_shin_X_fr_L_thigh();
                const Type_fr_L_shin_X_fr_L_thigh& update(const state_t&);
            };

            struct Type_fr_L_thigh_X_fr_L_shin : public TransformForce<Type_fr_L_thigh_X_fr_L_shin> {
                Type_fr_L_thigh_X_fr_L_shin();
                const Type_fr_L_thigh_X_fr_L_shin& update(const state_t&);
            };

            struct Type_fr_R_hip_X_fr_trunk : public TransformForce<Type_fr_R_hip_X_fr_trunk> {
                Type_fr_R_hip_X_fr_trunk();
                const Type_fr_R_hip_X_fr_trunk& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_hip : public TransformForce<Type_fr_trunk_X_fr_R_hip> {
                Type_fr_trunk_X_fr_R_hip();
                const Type_fr_trunk_X_fr_R_hip& update(const state_t&);
            };

            struct Type_fr_R_thigh_X_fr_R_hip : public TransformForce<Type_fr_R_thigh_X_fr_R_hip> {
                Type_fr_R_thigh_X_fr_R_hip();
                const Type_fr_R_thigh_X_fr_R_hip& update(const state_t&);
            };

            struct Type_fr_R_hip_X_fr_R_thigh : public TransformForce<Type_fr_R_hip_X_fr_R_thigh> {
                Type_fr_R_hip_X_fr_R_thigh();
                const Type_fr_R_hip_X_fr_R_thigh& update(const state_t&);
            };

            struct Type_fr_R_shin_X_fr_R_thigh : public TransformForce<Type_fr_R_shin_X_fr_R_thigh> {
                Type_fr_R_shin_X_fr_R_thigh();
                const Type_fr_R_shin_X_fr_R_thigh& update(const state_t&);
            };

            struct Type_fr_R_thigh_X_fr_R_shin : public TransformForce<Type_fr_R_thigh_X_fr_R_shin> {
                Type_fr_R_thigh_X_fr_R_shin();
                const Type_fr_R_thigh_X_fr_R_shin& update(const state_t&);
            };

        public:
            ForceTransforms();
            void updateParams(const Params_lengths&, const Params_angles&);

            Type_fr_trunk_X_L_foot fr_trunk_X_L_foot;
            Type_fr_trunk_X_fr_L_HAA fr_trunk_X_fr_L_HAA;
            Type_fr_trunk_X_fr_L_HFE fr_trunk_X_fr_L_HFE;
            Type_fr_trunk_X_fr_L_KFE fr_trunk_X_fr_L_KFE;
            Type_fr_trunk_X_R_foot fr_trunk_X_R_foot;
            Type_fr_trunk_X_fr_R_HAA fr_trunk_X_fr_R_HAA;
            Type_fr_trunk_X_fr_R_HFE fr_trunk_X_fr_R_HFE;
            Type_fr_trunk_X_fr_R_KFE fr_trunk_X_fr_R_KFE;
            Type_fr_L_hip_X_fr_trunk fr_L_hip_X_fr_trunk;
            Type_fr_trunk_X_fr_L_hip fr_trunk_X_fr_L_hip;
            Type_fr_L_thigh_X_fr_L_hip fr_L_thigh_X_fr_L_hip;
            Type_fr_L_hip_X_fr_L_thigh fr_L_hip_X_fr_L_thigh;
            Type_fr_L_shin_X_fr_L_thigh fr_L_shin_X_fr_L_thigh;
            Type_fr_L_thigh_X_fr_L_shin fr_L_thigh_X_fr_L_shin;
            Type_fr_R_hip_X_fr_trunk fr_R_hip_X_fr_trunk;
            Type_fr_trunk_X_fr_R_hip fr_trunk_X_fr_R_hip;
            Type_fr_R_thigh_X_fr_R_hip fr_R_thigh_X_fr_R_hip;
            Type_fr_R_hip_X_fr_R_thigh fr_R_hip_X_fr_R_thigh;
            Type_fr_R_shin_X_fr_R_thigh fr_R_shin_X_fr_R_thigh;
            Type_fr_R_thigh_X_fr_R_shin fr_R_thigh_X_fr_R_shin;

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

            struct Type_fr_trunk_X_L_foot : public TransformHomogeneous<Type_fr_trunk_X_L_foot> {
                Type_fr_trunk_X_L_foot();
                const Type_fr_trunk_X_L_foot& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_HAA : public TransformHomogeneous<Type_fr_trunk_X_fr_L_HAA> {
                Type_fr_trunk_X_fr_L_HAA();
                const Type_fr_trunk_X_fr_L_HAA& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_HFE : public TransformHomogeneous<Type_fr_trunk_X_fr_L_HFE> {
                Type_fr_trunk_X_fr_L_HFE();
                const Type_fr_trunk_X_fr_L_HFE& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_KFE : public TransformHomogeneous<Type_fr_trunk_X_fr_L_KFE> {
                Type_fr_trunk_X_fr_L_KFE();
                const Type_fr_trunk_X_fr_L_KFE& update(const state_t&);
            };

            struct Type_fr_trunk_X_R_foot : public TransformHomogeneous<Type_fr_trunk_X_R_foot> {
                Type_fr_trunk_X_R_foot();
                const Type_fr_trunk_X_R_foot& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_HAA : public TransformHomogeneous<Type_fr_trunk_X_fr_R_HAA> {
                Type_fr_trunk_X_fr_R_HAA();
                const Type_fr_trunk_X_fr_R_HAA& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_HFE : public TransformHomogeneous<Type_fr_trunk_X_fr_R_HFE> {
                Type_fr_trunk_X_fr_R_HFE();
                const Type_fr_trunk_X_fr_R_HFE& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_KFE : public TransformHomogeneous<Type_fr_trunk_X_fr_R_KFE> {
                Type_fr_trunk_X_fr_R_KFE();
                const Type_fr_trunk_X_fr_R_KFE& update(const state_t&);
            };

            struct Type_fr_L_hip_X_fr_trunk : public TransformHomogeneous<Type_fr_L_hip_X_fr_trunk> {
                Type_fr_L_hip_X_fr_trunk();
                const Type_fr_L_hip_X_fr_trunk& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_L_hip : public TransformHomogeneous<Type_fr_trunk_X_fr_L_hip> {
                Type_fr_trunk_X_fr_L_hip();
                const Type_fr_trunk_X_fr_L_hip& update(const state_t&);
            };

            struct Type_fr_L_thigh_X_fr_L_hip : public TransformHomogeneous<Type_fr_L_thigh_X_fr_L_hip> {
                Type_fr_L_thigh_X_fr_L_hip();
                const Type_fr_L_thigh_X_fr_L_hip& update(const state_t&);
            };

            struct Type_fr_L_hip_X_fr_L_thigh : public TransformHomogeneous<Type_fr_L_hip_X_fr_L_thigh> {
                Type_fr_L_hip_X_fr_L_thigh();
                const Type_fr_L_hip_X_fr_L_thigh& update(const state_t&);
            };

            struct Type_fr_L_shin_X_fr_L_thigh : public TransformHomogeneous<Type_fr_L_shin_X_fr_L_thigh> {
                Type_fr_L_shin_X_fr_L_thigh();
                const Type_fr_L_shin_X_fr_L_thigh& update(const state_t&);
            };

            struct Type_fr_L_thigh_X_fr_L_shin : public TransformHomogeneous<Type_fr_L_thigh_X_fr_L_shin> {
                Type_fr_L_thigh_X_fr_L_shin();
                const Type_fr_L_thigh_X_fr_L_shin& update(const state_t&);
            };

            struct Type_fr_R_hip_X_fr_trunk : public TransformHomogeneous<Type_fr_R_hip_X_fr_trunk> {
                Type_fr_R_hip_X_fr_trunk();
                const Type_fr_R_hip_X_fr_trunk& update(const state_t&);
            };

            struct Type_fr_trunk_X_fr_R_hip : public TransformHomogeneous<Type_fr_trunk_X_fr_R_hip> {
                Type_fr_trunk_X_fr_R_hip();
                const Type_fr_trunk_X_fr_R_hip& update(const state_t&);
            };

            struct Type_fr_R_thigh_X_fr_R_hip : public TransformHomogeneous<Type_fr_R_thigh_X_fr_R_hip> {
                Type_fr_R_thigh_X_fr_R_hip();
                const Type_fr_R_thigh_X_fr_R_hip& update(const state_t&);
            };

            struct Type_fr_R_hip_X_fr_R_thigh : public TransformHomogeneous<Type_fr_R_hip_X_fr_R_thigh> {
                Type_fr_R_hip_X_fr_R_thigh();
                const Type_fr_R_hip_X_fr_R_thigh& update(const state_t&);
            };

            struct Type_fr_R_shin_X_fr_R_thigh : public TransformHomogeneous<Type_fr_R_shin_X_fr_R_thigh> {
                Type_fr_R_shin_X_fr_R_thigh();
                const Type_fr_R_shin_X_fr_R_thigh& update(const state_t&);
            };

            struct Type_fr_R_thigh_X_fr_R_shin : public TransformHomogeneous<Type_fr_R_thigh_X_fr_R_shin> {
                Type_fr_R_thigh_X_fr_R_shin();
                const Type_fr_R_thigh_X_fr_R_shin& update(const state_t&);
            };

        public:
            HomogeneousTransforms();
            void updateParams(const Params_lengths&, const Params_angles&);

            Type_fr_trunk_X_L_foot fr_trunk_X_L_foot;
            Type_fr_trunk_X_fr_L_HAA fr_trunk_X_fr_L_HAA;
            Type_fr_trunk_X_fr_L_HFE fr_trunk_X_fr_L_HFE;
            Type_fr_trunk_X_fr_L_KFE fr_trunk_X_fr_L_KFE;
            Type_fr_trunk_X_R_foot fr_trunk_X_R_foot;
            Type_fr_trunk_X_fr_R_HAA fr_trunk_X_fr_R_HAA;
            Type_fr_trunk_X_fr_R_HFE fr_trunk_X_fr_R_HFE;
            Type_fr_trunk_X_fr_R_KFE fr_trunk_X_fr_R_KFE;
            Type_fr_L_hip_X_fr_trunk fr_L_hip_X_fr_trunk;
            Type_fr_trunk_X_fr_L_hip fr_trunk_X_fr_L_hip;
            Type_fr_L_thigh_X_fr_L_hip fr_L_thigh_X_fr_L_hip;
            Type_fr_L_hip_X_fr_L_thigh fr_L_hip_X_fr_L_thigh;
            Type_fr_L_shin_X_fr_L_thigh fr_L_shin_X_fr_L_thigh;
            Type_fr_L_thigh_X_fr_L_shin fr_L_thigh_X_fr_L_shin;
            Type_fr_R_hip_X_fr_trunk fr_R_hip_X_fr_trunk;
            Type_fr_trunk_X_fr_R_hip fr_trunk_X_fr_R_hip;
            Type_fr_R_thigh_X_fr_R_hip fr_R_thigh_X_fr_R_hip;
            Type_fr_R_hip_X_fr_R_thigh fr_R_hip_X_fr_R_thigh;
            Type_fr_R_shin_X_fr_R_thigh fr_R_shin_X_fr_R_thigh;
            Type_fr_R_thigh_X_fr_R_shin fr_R_thigh_X_fr_R_shin;

        protected:
            Parameters params;

        }; //class 'HomogeneousTransforms'

    }
}

#endif
