#ifndef RCG_HOPPER_INVERSE_DYNAMICS_H_
#define RCG_HOPPER_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace Hopper {
namespace rcg {

/**
 * The Inverse Dynamics routine for the robot Hopper.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */
    class InverseDynamics {
    public:
        typedef LinkDataMap<Force> ExtForces;

        /**
         * Default constructor
         * \param in the inertia properties of the links
         * \param tr the container of all the spatial motion transforms of
         *     the robot Hopper, which will be used by this instance
         *     to compute inverse-dynamics.
         */
        InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

        /** \name Inverse dynamics
         * The full Newton-Euler algorithm for the inverse dynamics of this robot.
         *
         * \param[out] jForces the joint force vector required to achieve the desired accelerations
         * \param[in] q the joint position vector
         * \param[in] qd the joint velocity vector
         * \param[in] qdd the desired joint acceleration vector
         * \param[in] fext the external forces acting on the links; this parameters
         *            defaults to zero
         */
        ///@{
        void id(
                JointState& jForces,
                const JointState& q, const JointState& qd, const JointState& qdd,
                const ExtForces& fext = zeroExtForces);
        void id(
                JointState& jForces,
                const JointState& qd, const JointState& qdd,
                const ExtForces& fext = zeroExtForces);
        ///@}

        /** \name Gravity terms
         * The joint forces (linear or rotational) required to compensate
         * for the effect of gravity, in a specific configuration.
         */
        ///@{
        void G_terms(JointState& jForces, const JointState& q);
        void G_terms(JointState& jForces);
        ///@}

        /** \name Centrifugal and Coriolis terms
         * The forces (linear or rotational) acting on the joints due to centrifugal and
         * Coriolis effects, for a specific configuration.
         */
        ///@{
        void C_terms(JointState& jForces, const JointState& q, const JointState& qd);
        void C_terms(JointState& jForces, const JointState& qd);
        ///@}
        /** Updates all the kinematics transforms used by the inverse dynamics routine. */
        void setJointStatus(const JointState& q) const;

    public:
        /** \name Getters
         * These functions return various spatial quantities used internally
         * by the inverse dynamics routines, like the spatial acceleration
         * of the links.
         *
         * The getters can be useful to retrieve the additional data that is not
         * returned explicitly by the inverse dynamics routines even though it
         * is computed. For example, after a call to the inverse dynamics,
         * the spatial velocity of all the links has been determined and
         * can be accessed.
         *
         * However, beware that certain routines might not use some of the
         * spatial quantities, which therefore would retain their last value
         * without being updated nor reset (for example, the spatial velocity
         * of the links is unaffected by the computation of the gravity terms).
         */
        ///@{
        const Velocity& getVelocity_u1() const { return u1_v; }
        const Acceleration& getAcceleration_u1() const { return u1_a; }
        const Force& getForce_u1() const { return u1_f; }
        const Velocity& getVelocity_u2() const { return u2_v; }
        const Acceleration& getAcceleration_u2() const { return u2_a; }
        const Force& getForce_u2() const { return u2_f; }
        const Velocity& getVelocity_body() const { return body_v; }
        const Acceleration& getAcceleration_body() const { return body_a; }
        const Force& getForce_body() const { return body_f; }
        const Velocity& getVelocity_leg() const { return leg_v; }
        const Acceleration& getAcceleration_leg() const { return leg_a; }
        const Force& getForce_leg() const { return leg_f; }
        ///@}
    protected:
        void firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext);
        void secondPass(JointState& jForces);

    private:
        InertiaProperties* inertiaProps;
        MotionTransforms* xm;
    private:
        Matrix66 vcross; // support variable
        // Link 'u1' :
        const InertiaMatrix& u1_I;
        Velocity u1_v;
        Acceleration u1_a;
        Force u1_f;
        // Link 'u2' :
        const InertiaMatrix& u2_I;
        Velocity u2_v;
        Acceleration u2_a;
        Force u2_f;
        // Link 'body' :
        const InertiaMatrix& body_I;
        Velocity body_v;
        Acceleration body_a;
        Force body_f;
        // Link 'leg' :
        const InertiaMatrix& leg_I;
        Velocity leg_v;
        Acceleration leg_a;
        Force leg_f;


    private:
        static const ExtForces zeroExtForces;
    };

    inline void InverseDynamics::setJointStatus(const JointState& q) const {
        (xm->fr_u1_X_fr_u0)(q);
        (xm->fr_u2_X_fr_u1)(q);
        (xm->fr_body_X_fr_u2)(q);
        (xm->fr_leg_X_fr_body)(q);
    }

    inline void InverseDynamics::G_terms(JointState& jForces, const JointState& q) {
        setJointStatus(q);
        G_terms(jForces);
    }

    inline void InverseDynamics::C_terms(JointState& jForces, const JointState& q, const JointState& qd) {
        setJointStatus(q);
        C_terms(jForces, qd);
    }

    inline void InverseDynamics::id(
            JointState& jForces,
            const JointState& q, const JointState& qd, const JointState& qdd,
            const ExtForces& fext) {
        setJointStatus(q);
        id(jForces, qd, qdd, fext);
    }

}
}

#endif
