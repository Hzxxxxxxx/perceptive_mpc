#ifndef RCG__RM_65_EXAMPLE_TRAITS_H_
#define RCG__RM_65_EXAMPLE_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace rm_65_example {
namespace rcg {
struct Traits {
    typedef typename rm_65_example::rcg::ScalarTraits ScalarTraits;

    typedef typename rm_65_example::rcg::JointState JointState;

    typedef typename rm_65_example::rcg::JointIdentifiers JointID;
    typedef typename rm_65_example::rcg::LinkIdentifiers  LinkID;

    typedef typename rm_65_example::rcg::HomogeneousTransforms HomogeneousTransforms;
    typedef typename rm_65_example::rcg::MotionTransforms MotionTransforms;
    typedef typename rm_65_example::rcg::ForceTransforms ForceTransforms;

    typedef typename rm_65_example::rcg::InertiaProperties InertiaProperties;
    typedef typename rm_65_example::rcg::ForwardDynamics FwdDynEngine;
    typedef typename rm_65_example::rcg::InverseDynamics InvDynEngine;
    typedef typename rm_65_example::rcg::JSIM JSIM;

    static const int joints_count = rm_65_example::rcg::jointsCount;
    static const int links_count  = rm_65_example::rcg::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return rm_65_example::rcg::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return rm_65_example::rcg::orderedLinkIDs;
}

}
}

#endif
