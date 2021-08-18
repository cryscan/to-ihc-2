#ifndef RCG_HOPPER_FORWARD_DYNAMICS_H_
#define RCG_HOPPER_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace Hopper {
    namespace rcg {

/**
 * The Forward Dynamics routine for the robot Hopper.
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
             *     the robot Hopper, which will be used by this instance
             *     to compute the dynamics.
             */
            ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
            /** \name Forward dynamics
             * The Articulated-Body-Algorithm to compute the joint accelerations
             */ ///@{
            /**
             * \param qdd the joint accelerations vector (output parameter).
             * \param q the joint status vector
             * \param qd the joint velocities vector
             * \param tau the joint forces (torque or force)
             * \param fext the external forces, optional. Each force must be
             *              expressed in the reference frame of the link it is
             *              exerted on.
             */
            void fd(
                    JointState& qdd, // output parameter
                    const JointState& q, const JointState& qd, const JointState& tau,
                    const ExtForces& fext = zeroExtForces);
            void fd(
                    JointState& qdd, // output parameter
                    const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
            ///@}

            /** Updates all the kinematics transforms used by this instance. */
            void setJointStatus(const JointState& q) const;

        private:
            InertiaProperties* inertiaProps;
            MotionTransforms* motionTransforms;

            Matrix66 vcross; // support variable
            Matrix66 Ia_p;   // support variable, articulated inertia in the case of a prismatic joint
            Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint

            // Link 'u1' :
            Matrix66 u1_AI;
            Velocity u1_a;
            Velocity u1_v;
            Velocity u1_c;
            Force u1_p;

            Column6 u1_U;
            Scalar u1_D;
            Scalar u1_u;
            // Link 'u2' :
            Matrix66 u2_AI;
            Velocity u2_a;
            Velocity u2_v;
            Velocity u2_c;
            Force u2_p;

            Column6 u2_U;
            Scalar u2_D;
            Scalar u2_u;
            // Link 'body' :
            Matrix66 body_AI;
            Velocity body_a;
            Velocity body_v;
            Velocity body_c;
            Force body_p;

            Column6 body_U;
            Scalar body_D;
            Scalar body_u;
            // Link 'leg' :
            Matrix66 leg_AI;
            Velocity leg_a;
            Velocity leg_v;
            Velocity leg_c;
            Force leg_p;

            Column6 leg_U;
            Scalar leg_D;
            Scalar leg_u;
        private:
            static const ExtForces zeroExtForces;
        };

        inline void ForwardDynamics::setJointStatus(const JointState& q) const {
            (motionTransforms->fr_u1_X_fr_u0)(q);
            (motionTransforms->fr_u2_X_fr_u1)(q);
            (motionTransforms->fr_body_X_fr_u2)(q);
            (motionTransforms->fr_leg_X_fr_body)(q);
        }

        inline void ForwardDynamics::fd(
                JointState& qdd,
                const JointState& q,
                const JointState& qd,
                const JointState& tau,
                const ExtForces& fext/* = zeroExtForces */) {
            setJointStatus(q);
            fd(qdd, qd, tau, fext);
        }

    }
}

#endif
