#ifndef RCG_RM_65_EXAMPLE_DECLARATIONS_H_
#define RCG_RM_65_EXAMPLE_DECLARATIONS_H_

// #include "rbd_types.h"

namespace rm_65_example {
namespace rcg {

static constexpr int JointSpaceDimension = 6;
static constexpr int jointsCount = 6;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 7;

typedef Matrix<6, 1> Column6d;
typedef Column6d JointState;

enum JointIdentifiers {
    JOINT1 = 0
    , JOINT2
    , JOINT3
    , JOINT4
    , JOINT5
    , JOINT6
};

enum LinkIdentifiers {
    BASE_LINK = 0
    , LINK1
    , LINK2
    , LINK3
    , LINK4
    , LINK5
    , LINK6
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {JOINT1,JOINT2,JOINT3,JOINT4,JOINT5,JOINT6};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE_LINK,LINK1,LINK2,LINK3,LINK4,LINK5,LINK6};

}
}
#endif
