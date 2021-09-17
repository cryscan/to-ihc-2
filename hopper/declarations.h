#ifndef RCG_HOPPER_DECLARATIONS_H_
#define RCG_HOPPER_DECLARATIONS_H_

#include "rbd_types.h"

namespace Hopper {
    namespace rcg {

        static constexpr int JointSpaceDimension = 4;
        static constexpr int jointsCount = 4;
/** The total number of rigid bodies of this robot, including the base */
        static constexpr int linksCount = 5;

        typedef Matrix<4, 1> Column4d;
        typedef Column4d JointState;

        enum JointIdentifiers {
            BH = 0, BX, HFE, KFE
        };

        enum LinkIdentifiers {
            U0 = 0, U1, U2, BODY, LEG
        };

        static const JointIdentifiers orderedJointIDs[jointsCount] =
                {BH, BX, HFE, KFE};

        static const LinkIdentifiers orderedLinkIDs[linksCount] =
                {U0, U1, U2, BODY, LEG};

    }
}
#endif
