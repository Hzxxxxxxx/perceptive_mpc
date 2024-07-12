#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const rm_65_example::rcg::ForwardDynamics::ExtForces
    rm_65_example::rcg::ForwardDynamics::zeroExtForces(Force::Zero());

rm_65_example::rcg::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    Link1_v.setZero();
    Link1_c.setZero();
    Link2_v.setZero();
    Link2_c.setZero();
    Link3_v.setZero();
    Link3_c.setZero();
    Link4_v.setZero();
    Link4_c.setZero();
    Link5_v.setZero();
    Link5_c.setZero();
    Link6_v.setZero();
    Link6_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void rm_65_example::rcg::ForwardDynamics::fd(
    JointState& qdd,
    Acceleration& base_link_a,
    const Velocity& base_link_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_link_AI = inertiaProps->getTensor_base_link();
    base_link_p = - fext[BASE_LINK];
    Link1_AI = inertiaProps->getTensor_Link1();
    Link1_p = - fext[LINK1];
    Link2_AI = inertiaProps->getTensor_Link2();
    Link2_p = - fext[LINK2];
    Link3_AI = inertiaProps->getTensor_Link3();
    Link3_p = - fext[LINK3];
    Link4_AI = inertiaProps->getTensor_Link4();
    Link4_p = - fext[LINK4];
    Link5_AI = inertiaProps->getTensor_Link5();
    Link5_p = - fext[LINK5];
    Link6_AI = inertiaProps->getTensor_Link6();
    Link6_p = - fext[LINK6];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link Link1
    //  - The spatial velocity:
    Link1_v = (motionTransforms-> fr_Link1_X_fr_base_link) * base_link_v;
    Link1_v(AZ) += qd(JOINT1);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(Link1_v, vcross);
    Link1_c = vcross.col(AZ) * qd(JOINT1);
    
    //  - The bias force term:
    Link1_p += vxIv(Link1_v, Link1_AI);
    
    // + Link Link2
    //  - The spatial velocity:
    Link2_v = (motionTransforms-> fr_Link2_X_fr_Link1) * Link1_v;
    Link2_v(AZ) += qd(JOINT2);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(Link2_v, vcross);
    Link2_c = vcross.col(AZ) * qd(JOINT2);
    
    //  - The bias force term:
    Link2_p += vxIv(Link2_v, Link2_AI);
    
    // + Link Link3
    //  - The spatial velocity:
    Link3_v = (motionTransforms-> fr_Link3_X_fr_Link2) * Link2_v;
    Link3_v(AZ) += qd(JOINT3);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(Link3_v, vcross);
    Link3_c = vcross.col(AZ) * qd(JOINT3);
    
    //  - The bias force term:
    Link3_p += vxIv(Link3_v, Link3_AI);
    
    // + Link Link4
    //  - The spatial velocity:
    Link4_v = (motionTransforms-> fr_Link4_X_fr_Link3) * Link3_v;
    Link4_v(AZ) += qd(JOINT4);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(Link4_v, vcross);
    Link4_c = vcross.col(AZ) * qd(JOINT4);
    
    //  - The bias force term:
    Link4_p += vxIv(Link4_v, Link4_AI);
    
    // + Link Link5
    //  - The spatial velocity:
    Link5_v = (motionTransforms-> fr_Link5_X_fr_Link4) * Link4_v;
    Link5_v(AZ) += qd(JOINT5);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(Link5_v, vcross);
    Link5_c = vcross.col(AZ) * qd(JOINT5);
    
    //  - The bias force term:
    Link5_p += vxIv(Link5_v, Link5_AI);
    
    // + Link Link6
    //  - The spatial velocity:
    Link6_v = (motionTransforms-> fr_Link6_X_fr_Link5) * Link5_v;
    Link6_v(AZ) += qd(JOINT6);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(Link6_v, vcross);
    Link6_c = vcross.col(AZ) * qd(JOINT6);
    
    //  - The bias force term:
    Link6_p += vxIv(Link6_v, Link6_AI);
    
    // + The floating base body
    base_link_p += vxIv(base_link_v, base_link_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;
    
    // + Link Link6
    Link6_u = tau(JOINT6) - Link6_p(AZ);
    Link6_U = Link6_AI.col(AZ);
    Link6_D = Link6_U(AZ);
    
    compute_Ia_revolute(Link6_AI, Link6_U, Link6_D, Ia_r);  // same as: Ia_r = Link6_AI - Link6_U/Link6_D * Link6_U.transpose();
    pa = Link6_p + Ia_r * Link6_c + Link6_U * Link6_u/Link6_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Link6_X_fr_Link5, IaB);
    Link5_AI += IaB;
    Link5_p += (motionTransforms-> fr_Link6_X_fr_Link5).transpose() * pa;
    
    // + Link Link5
    Link5_u = tau(JOINT5) - Link5_p(AZ);
    Link5_U = Link5_AI.col(AZ);
    Link5_D = Link5_U(AZ);
    
    compute_Ia_revolute(Link5_AI, Link5_U, Link5_D, Ia_r);  // same as: Ia_r = Link5_AI - Link5_U/Link5_D * Link5_U.transpose();
    pa = Link5_p + Ia_r * Link5_c + Link5_U * Link5_u/Link5_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Link5_X_fr_Link4, IaB);
    Link4_AI += IaB;
    Link4_p += (motionTransforms-> fr_Link5_X_fr_Link4).transpose() * pa;
    
    // + Link Link4
    Link4_u = tau(JOINT4) - Link4_p(AZ);
    Link4_U = Link4_AI.col(AZ);
    Link4_D = Link4_U(AZ);
    
    compute_Ia_revolute(Link4_AI, Link4_U, Link4_D, Ia_r);  // same as: Ia_r = Link4_AI - Link4_U/Link4_D * Link4_U.transpose();
    pa = Link4_p + Ia_r * Link4_c + Link4_U * Link4_u/Link4_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Link4_X_fr_Link3, IaB);
    Link3_AI += IaB;
    Link3_p += (motionTransforms-> fr_Link4_X_fr_Link3).transpose() * pa;
    
    // + Link Link3
    Link3_u = tau(JOINT3) - Link3_p(AZ);
    Link3_U = Link3_AI.col(AZ);
    Link3_D = Link3_U(AZ);
    
    compute_Ia_revolute(Link3_AI, Link3_U, Link3_D, Ia_r);  // same as: Ia_r = Link3_AI - Link3_U/Link3_D * Link3_U.transpose();
    pa = Link3_p + Ia_r * Link3_c + Link3_U * Link3_u/Link3_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Link3_X_fr_Link2, IaB);
    Link2_AI += IaB;
    Link2_p += (motionTransforms-> fr_Link3_X_fr_Link2).transpose() * pa;
    
    // + Link Link2
    Link2_u = tau(JOINT2) - Link2_p(AZ);
    Link2_U = Link2_AI.col(AZ);
    Link2_D = Link2_U(AZ);
    
    compute_Ia_revolute(Link2_AI, Link2_U, Link2_D, Ia_r);  // same as: Ia_r = Link2_AI - Link2_U/Link2_D * Link2_U.transpose();
    pa = Link2_p + Ia_r * Link2_c + Link2_U * Link2_u/Link2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Link2_X_fr_Link1, IaB);
    Link1_AI += IaB;
    Link1_p += (motionTransforms-> fr_Link2_X_fr_Link1).transpose() * pa;
    
    // + Link Link1
    Link1_u = tau(JOINT1) - Link1_p(AZ);
    Link1_U = Link1_AI.col(AZ);
    Link1_D = Link1_U(AZ);
    
    compute_Ia_revolute(Link1_AI, Link1_U, Link1_D, Ia_r);  // same as: Ia_r = Link1_AI - Link1_U/Link1_D * Link1_U.transpose();
    pa = Link1_p + Ia_r * Link1_c + Link1_U * Link1_u/Link1_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_Link1_X_fr_base_link, IaB);
    base_link_AI += IaB;
    base_link_p += (motionTransforms-> fr_Link1_X_fr_base_link).transpose() * pa;
    
    // + The acceleration of the floating base base_link, without gravity
    base_link_a = - base_link_AI.llt().solve(base_link_p);  // base_link_a = - IA^-1 * base_link_p
    
    // ---------------------- THIRD PASS ---------------------- //
    Link1_a = (motionTransforms-> fr_Link1_X_fr_base_link) * base_link_a + Link1_c;
    qdd(JOINT1) = (Link1_u - Link1_U.dot(Link1_a)) / Link1_D;
    Link1_a(AZ) += qdd(JOINT1);
    
    Link2_a = (motionTransforms-> fr_Link2_X_fr_Link1) * Link1_a + Link2_c;
    qdd(JOINT2) = (Link2_u - Link2_U.dot(Link2_a)) / Link2_D;
    Link2_a(AZ) += qdd(JOINT2);
    
    Link3_a = (motionTransforms-> fr_Link3_X_fr_Link2) * Link2_a + Link3_c;
    qdd(JOINT3) = (Link3_u - Link3_U.dot(Link3_a)) / Link3_D;
    Link3_a(AZ) += qdd(JOINT3);
    
    Link4_a = (motionTransforms-> fr_Link4_X_fr_Link3) * Link3_a + Link4_c;
    qdd(JOINT4) = (Link4_u - Link4_U.dot(Link4_a)) / Link4_D;
    Link4_a(AZ) += qdd(JOINT4);
    
    Link5_a = (motionTransforms-> fr_Link5_X_fr_Link4) * Link4_a + Link5_c;
    qdd(JOINT5) = (Link5_u - Link5_U.dot(Link5_a)) / Link5_D;
    Link5_a(AZ) += qdd(JOINT5);
    
    Link6_a = (motionTransforms-> fr_Link6_X_fr_Link5) * Link5_a + Link6_c;
    qdd(JOINT6) = (Link6_u - Link6_U.dot(Link6_a)) / Link6_D;
    Link6_a(AZ) += qdd(JOINT6);
    
    
    // + Add gravity to the acceleration of the floating base
    base_link_a += g;
}
