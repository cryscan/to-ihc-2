#ifndef RCG_BIPED_DECLARATIONS_H_
#define RCG_BIPED_DECLARATIONS_H_

#include "rbd_types.h"

namespace Biped {
    namespace rcg {

        static constexpr int JointSpaceDimension = 6;
        static constexpr int jointsCount = 6;
/** The total number of rigid bodies of this robot, including the base */
        static constexpr int linksCount = 7;

        typedef Matrix<6, 1> Column6d;
        typedef Column6d JointState;

        enum JointIdentifiers {
            L_HAA = 0, L_HFE, L_KFE, R_HAA, R_HFE, R_KFE
        };

        enum LinkIdentifiers {
            TRUNK = 0, L_HIP, L_THIGH, L_SHIN, R_HIP, R_THIGH, R_SHIN
        };

        static const JointIdentifiers orderedJointIDs[jointsCount] =
                {L_HAA, L_HFE, L_KFE, R_HAA, R_HFE, R_KFE};

        static const LinkIdentifiers orderedLinkIDs[linksCount] =
                {TRUNK, L_HIP, L_THIGH, L_SHIN, R_HIP, R_THIGH, R_SHIN};

    }
}
#endif
