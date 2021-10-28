#ifndef RCG_BIPED_FORWARD_DYNAMICS_H_
#define RCG_BIPED_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace Biped {
    namespace rcg {

/**
 * The Forward Dynamics routine for the robot Biped.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
        class ForwardDynamics {
        public:
            typedef LinkDataMap<Force> ExtForces;

            /**
             * Default constructor
             * \param in the inertia properties of the links
             * \param tr the container of all the spatial motion transforms of
             *     the robot Biped, which will be used by this instance
             *     to compute the dynamics.
             */
            ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
            /** \name Forward dynamics
             * The Articulated-Body-Algorithm to compute the joint accelerations
             */ ///@{
            /**
             * \param qdd the joint accelerations vector (output parameter).
             * \param trunk_a
             * \param trunk_v
             * \param g the gravity acceleration vector, expressed in the
             *          base coordinates
             * \param q the joint status vector
             * \param qd the joint velocities vector
             * \param tau the joint forces (torque or force)
             * \param fext the external forces, optional. Each force must be
             *              expressed in the reference frame of the link it is
             *              exerted on.
             */
            void fd(
                    JointState& qdd, Acceleration& trunk_a, // output parameters,
                    const Velocity& trunk_v, const Acceleration& g,
                    const JointState& q, const JointState& qd, const JointState& tau,
                    const ExtForces& fext = zeroExtForces);
            void fd(
                    JointState& qdd, Acceleration& trunk_a, // output parameters,
                    const Velocity& trunk_v, const Acceleration& g,
                    const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
            ///@}

            /** Updates all the kinematics transforms used by this instance. */
            void setJointStatus(const JointState& q) const;

        private:
            InertiaProperties* inertiaProps;
            MotionTransforms* motionTransforms;

            Matrix66 vcross; // support variable
            Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint
            // Link 'trunk'
            Matrix66 trunk_AI;
            Force trunk_p;

            // Link 'L_hip' :
            Matrix66 L_hip_AI;
            Velocity L_hip_a;
            Velocity L_hip_v;
            Velocity L_hip_c;
            Force L_hip_p;

            Column6 L_hip_U;
            Scalar L_hip_D;
            Scalar L_hip_u;
            // Link 'L_thigh' :
            Matrix66 L_thigh_AI;
            Velocity L_thigh_a;
            Velocity L_thigh_v;
            Velocity L_thigh_c;
            Force L_thigh_p;

            Column6 L_thigh_U;
            Scalar L_thigh_D;
            Scalar L_thigh_u;
            // Link 'L_shin' :
            Matrix66 L_shin_AI;
            Velocity L_shin_a;
            Velocity L_shin_v;
            Velocity L_shin_c;
            Force L_shin_p;

            Column6 L_shin_U;
            Scalar L_shin_D;
            Scalar L_shin_u;
            // Link 'R_hip' :
            Matrix66 R_hip_AI;
            Velocity R_hip_a;
            Velocity R_hip_v;
            Velocity R_hip_c;
            Force R_hip_p;

            Column6 R_hip_U;
            Scalar R_hip_D;
            Scalar R_hip_u;
            // Link 'R_thigh' :
            Matrix66 R_thigh_AI;
            Velocity R_thigh_a;
            Velocity R_thigh_v;
            Velocity R_thigh_c;
            Force R_thigh_p;

            Column6 R_thigh_U;
            Scalar R_thigh_D;
            Scalar R_thigh_u;
            // Link 'R_shin' :
            Matrix66 R_shin_AI;
            Velocity R_shin_a;
            Velocity R_shin_v;
            Velocity R_shin_c;
            Force R_shin_p;

            Column6 R_shin_U;
            Scalar R_shin_D;
            Scalar R_shin_u;
        private:
            static const ExtForces zeroExtForces;
        };

        inline void ForwardDynamics::setJointStatus(const JointState& q) const {
            (motionTransforms->fr_L_hip_X_fr_trunk)(q);
            (motionTransforms->fr_L_thigh_X_fr_L_hip)(q);
            (motionTransforms->fr_L_shin_X_fr_L_thigh)(q);
            (motionTransforms->fr_R_hip_X_fr_trunk)(q);
            (motionTransforms->fr_R_thigh_X_fr_R_hip)(q);
            (motionTransforms->fr_R_shin_X_fr_R_thigh)(q);
        }

        inline void ForwardDynamics::fd(
                JointState& qdd, Acceleration& trunk_a, // output parameters,
                const Velocity& trunk_v, const Acceleration& g,
                const JointState& q,
                const JointState& qd,
                const JointState& tau,
                const ExtForces& fext/* = zeroExtForces */) {
            setJointStatus(q);
            fd(qdd, trunk_a, trunk_v, g, qd, tau, fext);
        }

    }
}

#endif
