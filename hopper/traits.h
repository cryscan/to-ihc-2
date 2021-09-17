#ifndef RCG__HOPPER_TRAITS_H_
#define RCG__HOPPER_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace Hopper {
    namespace rcg {
        struct Traits {
            typedef typename Hopper::rcg::ScalarTraits ScalarTraits;

            typedef typename Hopper::rcg::JointState JointState;

            typedef typename Hopper::rcg::JointIdentifiers JointID;
            typedef typename Hopper::rcg::LinkIdentifiers LinkID;

            typedef typename Hopper::rcg::HomogeneousTransforms HomogeneousTransforms;
            typedef typename Hopper::rcg::MotionTransforms MotionTransforms;
            typedef typename Hopper::rcg::ForceTransforms ForceTransforms;

            typedef typename Hopper::rcg::InertiaProperties InertiaProperties;
            typedef typename Hopper::rcg::ForwardDynamics FwdDynEngine;
            typedef typename Hopper::rcg::InverseDynamics InvDynEngine;
            typedef typename Hopper::rcg::JSIM JSIM;

            static const int joints_count = Hopper::rcg::jointsCount;
            static const int links_count = Hopper::rcg::linksCount;
            static const bool floating_base = false;

            static inline const JointID* orderedJointIDs();
            static inline const LinkID* orderedLinkIDs();
        };


        inline const Traits::JointID* Traits::orderedJointIDs() {
            return Hopper::rcg::orderedJointIDs;
        }
        inline const Traits::LinkID* Traits::orderedLinkIDs() {
            return Hopper::rcg::orderedLinkIDs;
        }

    }
}

#endif
