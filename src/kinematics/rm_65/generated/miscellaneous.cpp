#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace rm_65_example::rcg;

Vector3 rm_65_example::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    Vector3 tmpSum(Vector3::Zero());

    tmpSum += inertiaProps.getCOM_base_link() * inertiaProps.getMass_base_link();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_base_link_X_fr_Link1;
    tmpSum += inertiaProps.getMass_Link1() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Link1()));
    
    tmpX = tmpX * ht.fr_Link1_X_fr_Link2;
    tmpSum += inertiaProps.getMass_Link2() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Link2()));
    
    tmpX = tmpX * ht.fr_Link2_X_fr_Link3;
    tmpSum += inertiaProps.getMass_Link3() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Link3()));
    
    tmpX = tmpX * ht.fr_Link3_X_fr_Link4;
    tmpSum += inertiaProps.getMass_Link4() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Link4()));
    
    tmpX = tmpX * ht.fr_Link4_X_fr_Link5;
    tmpSum += inertiaProps.getMass_Link5() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Link5()));
    
    tmpX = tmpX * ht.fr_Link5_X_fr_Link6;
    tmpSum += inertiaProps.getMass_Link6() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_Link6()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 rm_65_example::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_link_X_fr_Link1(q);
    ht.fr_Link1_X_fr_Link2(q);
    ht.fr_Link2_X_fr_Link3(q);
    ht.fr_Link3_X_fr_Link4(q);
    ht.fr_Link4_X_fr_Link5(q);
    ht.fr_Link5_X_fr_Link6(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
