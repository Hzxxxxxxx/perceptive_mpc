#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace rm_65_example::rcg;

// Initialization of static-const data
const rm_65_example::rcg::InverseDynamics::ExtForces
rm_65_example::rcg::InverseDynamics::zeroExtForces(Force::Zero());

rm_65_example::rcg::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    Link1_I(inertiaProps->getTensor_Link1() ),
    Link2_I(inertiaProps->getTensor_Link2() ),
    Link3_I(inertiaProps->getTensor_Link3() ),
    Link4_I(inertiaProps->getTensor_Link4() ),
    Link5_I(inertiaProps->getTensor_Link5() ),
    Link6_I(inertiaProps->getTensor_Link6() )
    ,
        base_link_I( inertiaProps->getTensor_base_link() ),
        Link6_Ic(Link6_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot rm_65_example, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    Link1_v.setZero();
    Link2_v.setZero();
    Link3_v.setZero();
    Link4_v.setZero();
    Link5_v.setZero();
    Link6_v.setZero();

    vcross.setZero();
}

void rm_65_example::rcg::InverseDynamics::id(
    JointState& jForces, Acceleration& base_link_a,
    const Acceleration& g, const Velocity& base_link_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_link_Ic = base_link_I;
    Link1_Ic = Link1_I;
    Link2_Ic = Link2_I;
    Link3_Ic = Link3_I;
    Link4_Ic = Link4_I;
    Link5_Ic = Link5_I;

    // First pass, link 'Link1'
    Link1_v = ((xm->fr_Link1_X_fr_base_link) * base_link_v);
    Link1_v(iit::rbd::AZ) += qd(JOINT1);
    
    motionCrossProductMx<Scalar>(Link1_v, vcross);
    
    Link1_a = (vcross.col(iit::rbd::AZ) * qd(JOINT1));
    Link1_a(iit::rbd::AZ) += qdd(JOINT1);
    
    Link1_f = Link1_I * Link1_a + vxIv(Link1_v, Link1_I);
    
    // First pass, link 'Link2'
    Link2_v = ((xm->fr_Link2_X_fr_Link1) * Link1_v);
    Link2_v(iit::rbd::AZ) += qd(JOINT2);
    
    motionCrossProductMx<Scalar>(Link2_v, vcross);
    
    Link2_a = (xm->fr_Link2_X_fr_Link1) * Link1_a + vcross.col(iit::rbd::AZ) * qd(JOINT2);
    Link2_a(iit::rbd::AZ) += qdd(JOINT2);
    
    Link2_f = Link2_I * Link2_a + vxIv(Link2_v, Link2_I);
    
    // First pass, link 'Link3'
    Link3_v = ((xm->fr_Link3_X_fr_Link2) * Link2_v);
    Link3_v(iit::rbd::AZ) += qd(JOINT3);
    
    motionCrossProductMx<Scalar>(Link3_v, vcross);
    
    Link3_a = (xm->fr_Link3_X_fr_Link2) * Link2_a + vcross.col(iit::rbd::AZ) * qd(JOINT3);
    Link3_a(iit::rbd::AZ) += qdd(JOINT3);
    
    Link3_f = Link3_I * Link3_a + vxIv(Link3_v, Link3_I);
    
    // First pass, link 'Link4'
    Link4_v = ((xm->fr_Link4_X_fr_Link3) * Link3_v);
    Link4_v(iit::rbd::AZ) += qd(JOINT4);
    
    motionCrossProductMx<Scalar>(Link4_v, vcross);
    
    Link4_a = (xm->fr_Link4_X_fr_Link3) * Link3_a + vcross.col(iit::rbd::AZ) * qd(JOINT4);
    Link4_a(iit::rbd::AZ) += qdd(JOINT4);
    
    Link4_f = Link4_I * Link4_a + vxIv(Link4_v, Link4_I);
    
    // First pass, link 'Link5'
    Link5_v = ((xm->fr_Link5_X_fr_Link4) * Link4_v);
    Link5_v(iit::rbd::AZ) += qd(JOINT5);
    
    motionCrossProductMx<Scalar>(Link5_v, vcross);
    
    Link5_a = (xm->fr_Link5_X_fr_Link4) * Link4_a + vcross.col(iit::rbd::AZ) * qd(JOINT5);
    Link5_a(iit::rbd::AZ) += qdd(JOINT5);
    
    Link5_f = Link5_I * Link5_a + vxIv(Link5_v, Link5_I);
    
    // First pass, link 'Link6'
    Link6_v = ((xm->fr_Link6_X_fr_Link5) * Link5_v);
    Link6_v(iit::rbd::AZ) += qd(JOINT6);
    
    motionCrossProductMx<Scalar>(Link6_v, vcross);
    
    Link6_a = (xm->fr_Link6_X_fr_Link5) * Link5_a + vcross.col(iit::rbd::AZ) * qd(JOINT6);
    Link6_a(iit::rbd::AZ) += qdd(JOINT6);
    
    Link6_f = Link6_I * Link6_a + vxIv(Link6_v, Link6_I);
    
    // The force exerted on the floating base by the links
    base_link_f = vxIv(base_link_v, base_link_I);
    

    // Add the external forces:
    base_link_f -= fext[BASE_LINK];
    Link1_f -= fext[LINK1];
    Link2_f -= fext[LINK2];
    Link3_f -= fext[LINK3];
    Link4_f -= fext[LINK4];
    Link5_f -= fext[LINK5];
    Link6_f -= fext[LINK6];

    InertiaMatrix Ic_spare;
    iit::rbd::transformInertia<Scalar>(Link6_Ic, (xm->fr_Link6_X_fr_Link5).transpose(), Ic_spare);
    Link5_Ic += Ic_spare;
    Link5_f = Link5_f + (xm->fr_Link6_X_fr_Link5).transpose() * Link6_f;
    
    iit::rbd::transformInertia<Scalar>(Link5_Ic, (xm->fr_Link5_X_fr_Link4).transpose(), Ic_spare);
    Link4_Ic += Ic_spare;
    Link4_f = Link4_f + (xm->fr_Link5_X_fr_Link4).transpose() * Link5_f;
    
    iit::rbd::transformInertia<Scalar>(Link4_Ic, (xm->fr_Link4_X_fr_Link3).transpose(), Ic_spare);
    Link3_Ic += Ic_spare;
    Link3_f = Link3_f + (xm->fr_Link4_X_fr_Link3).transpose() * Link4_f;
    
    iit::rbd::transformInertia<Scalar>(Link3_Ic, (xm->fr_Link3_X_fr_Link2).transpose(), Ic_spare);
    Link2_Ic += Ic_spare;
    Link2_f = Link2_f + (xm->fr_Link3_X_fr_Link2).transpose() * Link3_f;
    
    iit::rbd::transformInertia<Scalar>(Link2_Ic, (xm->fr_Link2_X_fr_Link1).transpose(), Ic_spare);
    Link1_Ic += Ic_spare;
    Link1_f = Link1_f + (xm->fr_Link2_X_fr_Link1).transpose() * Link2_f;
    
    iit::rbd::transformInertia<Scalar>(Link1_Ic, (xm->fr_Link1_X_fr_base_link).transpose(), Ic_spare);
    base_link_Ic += Ic_spare;
    base_link_f = base_link_f + (xm->fr_Link1_X_fr_base_link).transpose() * Link1_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_link_a = - base_link_Ic.inverse() * base_link_f;
    
    Link1_a = xm->fr_Link1_X_fr_base_link * base_link_a;
    jForces(JOINT1) = (Link1_Ic.row(iit::rbd::AZ) * Link1_a + Link1_f(iit::rbd::AZ));
    
    Link2_a = xm->fr_Link2_X_fr_Link1 * Link1_a;
    jForces(JOINT2) = (Link2_Ic.row(iit::rbd::AZ) * Link2_a + Link2_f(iit::rbd::AZ));
    
    Link3_a = xm->fr_Link3_X_fr_Link2 * Link2_a;
    jForces(JOINT3) = (Link3_Ic.row(iit::rbd::AZ) * Link3_a + Link3_f(iit::rbd::AZ));
    
    Link4_a = xm->fr_Link4_X_fr_Link3 * Link3_a;
    jForces(JOINT4) = (Link4_Ic.row(iit::rbd::AZ) * Link4_a + Link4_f(iit::rbd::AZ));
    
    Link5_a = xm->fr_Link5_X_fr_Link4 * Link4_a;
    jForces(JOINT5) = (Link5_Ic.row(iit::rbd::AZ) * Link5_a + Link5_f(iit::rbd::AZ));
    
    Link6_a = xm->fr_Link6_X_fr_Link5 * Link5_a;
    jForces(JOINT6) = (Link6_Ic.row(iit::rbd::AZ) * Link6_a + Link6_f(iit::rbd::AZ));
    

    base_link_a += g;
}


void rm_65_example::rcg::InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_link_a = -g;

    // Link 'Link1'
    Link1_a = (xm->fr_Link1_X_fr_base_link) * base_link_a;
    Link1_f = Link1_I * Link1_a;
    // Link 'Link2'
    Link2_a = (xm->fr_Link2_X_fr_Link1) * Link1_a;
    Link2_f = Link2_I * Link2_a;
    // Link 'Link3'
    Link3_a = (xm->fr_Link3_X_fr_Link2) * Link2_a;
    Link3_f = Link3_I * Link3_a;
    // Link 'Link4'
    Link4_a = (xm->fr_Link4_X_fr_Link3) * Link3_a;
    Link4_f = Link4_I * Link4_a;
    // Link 'Link5'
    Link5_a = (xm->fr_Link5_X_fr_Link4) * Link4_a;
    Link5_f = Link5_I * Link5_a;
    // Link 'Link6'
    Link6_a = (xm->fr_Link6_X_fr_Link5) * Link5_a;
    Link6_f = Link6_I * Link6_a;

    base_link_f = base_link_I * base_link_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

void rm_65_example::rcg::InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_v, const JointState& qd)
{
    // Link 'Link1'
    Link1_v = ((xm->fr_Link1_X_fr_base_link) * base_link_v);
    Link1_v(iit::rbd::AZ) += qd(JOINT1);
    motionCrossProductMx<Scalar>(Link1_v, vcross);
    Link1_a = (vcross.col(iit::rbd::AZ) * qd(JOINT1));
    Link1_f = Link1_I * Link1_a + vxIv(Link1_v, Link1_I);
    
    // Link 'Link2'
    Link2_v = ((xm->fr_Link2_X_fr_Link1) * Link1_v);
    Link2_v(iit::rbd::AZ) += qd(JOINT2);
    motionCrossProductMx<Scalar>(Link2_v, vcross);
    Link2_a = (xm->fr_Link2_X_fr_Link1) * Link1_a + vcross.col(iit::rbd::AZ) * qd(JOINT2);
    Link2_f = Link2_I * Link2_a + vxIv(Link2_v, Link2_I);
    
    // Link 'Link3'
    Link3_v = ((xm->fr_Link3_X_fr_Link2) * Link2_v);
    Link3_v(iit::rbd::AZ) += qd(JOINT3);
    motionCrossProductMx<Scalar>(Link3_v, vcross);
    Link3_a = (xm->fr_Link3_X_fr_Link2) * Link2_a + vcross.col(iit::rbd::AZ) * qd(JOINT3);
    Link3_f = Link3_I * Link3_a + vxIv(Link3_v, Link3_I);
    
    // Link 'Link4'
    Link4_v = ((xm->fr_Link4_X_fr_Link3) * Link3_v);
    Link4_v(iit::rbd::AZ) += qd(JOINT4);
    motionCrossProductMx<Scalar>(Link4_v, vcross);
    Link4_a = (xm->fr_Link4_X_fr_Link3) * Link3_a + vcross.col(iit::rbd::AZ) * qd(JOINT4);
    Link4_f = Link4_I * Link4_a + vxIv(Link4_v, Link4_I);
    
    // Link 'Link5'
    Link5_v = ((xm->fr_Link5_X_fr_Link4) * Link4_v);
    Link5_v(iit::rbd::AZ) += qd(JOINT5);
    motionCrossProductMx<Scalar>(Link5_v, vcross);
    Link5_a = (xm->fr_Link5_X_fr_Link4) * Link4_a + vcross.col(iit::rbd::AZ) * qd(JOINT5);
    Link5_f = Link5_I * Link5_a + vxIv(Link5_v, Link5_I);
    
    // Link 'Link6'
    Link6_v = ((xm->fr_Link6_X_fr_Link5) * Link5_v);
    Link6_v(iit::rbd::AZ) += qd(JOINT6);
    motionCrossProductMx<Scalar>(Link6_v, vcross);
    Link6_a = (xm->fr_Link6_X_fr_Link5) * Link5_a + vcross.col(iit::rbd::AZ) * qd(JOINT6);
    Link6_f = Link6_I * Link6_a + vxIv(Link6_v, Link6_I);
    

    base_link_f = vxIv(base_link_v, base_link_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

void rm_65_example::rcg::InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_link_a = baseAccel -g;

    // First pass, link 'Link1'
    Link1_v = ((xm->fr_Link1_X_fr_base_link) * base_link_v);
    Link1_v(iit::rbd::AZ) += qd(JOINT1);
    
    motionCrossProductMx<Scalar>(Link1_v, vcross);
    
    Link1_a = (xm->fr_Link1_X_fr_base_link) * base_link_a + vcross.col(iit::rbd::AZ) * qd(JOINT1);
    Link1_a(iit::rbd::AZ) += qdd(JOINT1);
    
    Link1_f = Link1_I * Link1_a + vxIv(Link1_v, Link1_I) - fext[LINK1];
    
    // First pass, link 'Link2'
    Link2_v = ((xm->fr_Link2_X_fr_Link1) * Link1_v);
    Link2_v(iit::rbd::AZ) += qd(JOINT2);
    
    motionCrossProductMx<Scalar>(Link2_v, vcross);
    
    Link2_a = (xm->fr_Link2_X_fr_Link1) * Link1_a + vcross.col(iit::rbd::AZ) * qd(JOINT2);
    Link2_a(iit::rbd::AZ) += qdd(JOINT2);
    
    Link2_f = Link2_I * Link2_a + vxIv(Link2_v, Link2_I) - fext[LINK2];
    
    // First pass, link 'Link3'
    Link3_v = ((xm->fr_Link3_X_fr_Link2) * Link2_v);
    Link3_v(iit::rbd::AZ) += qd(JOINT3);
    
    motionCrossProductMx<Scalar>(Link3_v, vcross);
    
    Link3_a = (xm->fr_Link3_X_fr_Link2) * Link2_a + vcross.col(iit::rbd::AZ) * qd(JOINT3);
    Link3_a(iit::rbd::AZ) += qdd(JOINT3);
    
    Link3_f = Link3_I * Link3_a + vxIv(Link3_v, Link3_I) - fext[LINK3];
    
    // First pass, link 'Link4'
    Link4_v = ((xm->fr_Link4_X_fr_Link3) * Link3_v);
    Link4_v(iit::rbd::AZ) += qd(JOINT4);
    
    motionCrossProductMx<Scalar>(Link4_v, vcross);
    
    Link4_a = (xm->fr_Link4_X_fr_Link3) * Link3_a + vcross.col(iit::rbd::AZ) * qd(JOINT4);
    Link4_a(iit::rbd::AZ) += qdd(JOINT4);
    
    Link4_f = Link4_I * Link4_a + vxIv(Link4_v, Link4_I) - fext[LINK4];
    
    // First pass, link 'Link5'
    Link5_v = ((xm->fr_Link5_X_fr_Link4) * Link4_v);
    Link5_v(iit::rbd::AZ) += qd(JOINT5);
    
    motionCrossProductMx<Scalar>(Link5_v, vcross);
    
    Link5_a = (xm->fr_Link5_X_fr_Link4) * Link4_a + vcross.col(iit::rbd::AZ) * qd(JOINT5);
    Link5_a(iit::rbd::AZ) += qdd(JOINT5);
    
    Link5_f = Link5_I * Link5_a + vxIv(Link5_v, Link5_I) - fext[LINK5];
    
    // First pass, link 'Link6'
    Link6_v = ((xm->fr_Link6_X_fr_Link5) * Link5_v);
    Link6_v(iit::rbd::AZ) += qd(JOINT6);
    
    motionCrossProductMx<Scalar>(Link6_v, vcross);
    
    Link6_a = (xm->fr_Link6_X_fr_Link5) * Link5_a + vcross.col(iit::rbd::AZ) * qd(JOINT6);
    Link6_a(iit::rbd::AZ) += qdd(JOINT6);
    
    Link6_f = Link6_I * Link6_a + vxIv(Link6_v, Link6_I) - fext[LINK6];
    

    // The base
    base_link_f = base_link_I * base_link_a + vxIv(base_link_v, base_link_I) - fext[BASE_LINK];

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}


void rm_65_example::rcg::InverseDynamics::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'Link6'
    jForces(JOINT6) = Link6_f(iit::rbd::AZ);
    Link5_f += xm->fr_Link6_X_fr_Link5.transpose() * Link6_f;
    // Link 'Link5'
    jForces(JOINT5) = Link5_f(iit::rbd::AZ);
    Link4_f += xm->fr_Link5_X_fr_Link4.transpose() * Link5_f;
    // Link 'Link4'
    jForces(JOINT4) = Link4_f(iit::rbd::AZ);
    Link3_f += xm->fr_Link4_X_fr_Link3.transpose() * Link4_f;
    // Link 'Link3'
    jForces(JOINT3) = Link3_f(iit::rbd::AZ);
    Link2_f += xm->fr_Link3_X_fr_Link2.transpose() * Link3_f;
    // Link 'Link2'
    jForces(JOINT2) = Link2_f(iit::rbd::AZ);
    Link1_f += xm->fr_Link2_X_fr_Link1.transpose() * Link2_f;
    // Link 'Link1'
    jForces(JOINT1) = Link1_f(iit::rbd::AZ);
    base_link_f += xm->fr_Link1_X_fr_base_link.transpose() * Link1_f;
}
