#ifndef RCG__BIPED_TRAITS_H_
#define RCG__BIPED_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace Biped {
    namespace rcg {
        struct Traits {
            typedef typename Biped::rcg::ScalarTraits ScalarTraits;

            typedef typename Biped::rcg::JointState JointState;

            typedef typename Biped::rcg::JointIdentifiers JointID;
            typedef typename Biped::rcg::LinkIdentifiers LinkID;

            typedef typename Biped::rcg::HomogeneousTransforms HomogeneousTransforms;
            typedef typename Biped::rcg::MotionTransforms MotionTransforms;
            typedef typename Biped::rcg::ForceTransforms ForceTransforms;

            typedef typename Biped::rcg::InertiaProperties InertiaProperties;
            typedef typename Biped::rcg::ForwardDynamics FwdDynEngine;
            typedef typename Biped::rcg::InverseDynamics InvDynEngine;
            typedef typename Biped::rcg::JSIM JSIM;

            static const int joints_count = Biped::rcg::jointsCount;
            static const int links_count = Biped::rcg::linksCount;
            static const bool floating_base = true;

            static inline const JointID* orderedJointIDs();
            static inline const LinkID* orderedLinkIDs();
        };


        inline const Traits::JointID* Traits::orderedJointIDs() {
            return Biped::rcg::orderedJointIDs;
        }
        inline const Traits::LinkID* Traits::orderedLinkIDs() {
            return Biped::rcg::orderedLinkIDs;
        }

    }
}

#endif
