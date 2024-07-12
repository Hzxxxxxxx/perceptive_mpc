#ifndef RCG_RM_65_EXAMPLE_FORWARD_DYNAMICS_H_
#define RCG_RM_65_EXAMPLE_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace rm_65_example {
namespace rcg {

/**
 * The Forward Dynamics routine for the robot rm_65_example.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot rm_65_example, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param base_link_a
     * \param base_link_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& base_link_a, // output parameters,
       const Velocity& base_link_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& base_link_a, // output parameters,
        const Velocity& base_link_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    Matrix66 vcross; // support variable
    Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'base_link'
    Matrix66 base_link_AI;
    Force base_link_p;

    // Link 'Link1' :
    Matrix66 Link1_AI;
    Velocity Link1_a;
    Velocity Link1_v;
    Velocity Link1_c;
    Force    Link1_p;

    Column6 Link1_U;
    Scalar Link1_D;
    Scalar Link1_u;
    // Link 'Link2' :
    Matrix66 Link2_AI;
    Velocity Link2_a;
    Velocity Link2_v;
    Velocity Link2_c;
    Force    Link2_p;

    Column6 Link2_U;
    Scalar Link2_D;
    Scalar Link2_u;
    // Link 'Link3' :
    Matrix66 Link3_AI;
    Velocity Link3_a;
    Velocity Link3_v;
    Velocity Link3_c;
    Force    Link3_p;

    Column6 Link3_U;
    Scalar Link3_D;
    Scalar Link3_u;
    // Link 'Link4' :
    Matrix66 Link4_AI;
    Velocity Link4_a;
    Velocity Link4_v;
    Velocity Link4_c;
    Force    Link4_p;

    Column6 Link4_U;
    Scalar Link4_D;
    Scalar Link4_u;
    // Link 'Link5' :
    Matrix66 Link5_AI;
    Velocity Link5_a;
    Velocity Link5_v;
    Velocity Link5_c;
    Force    Link5_p;

    Column6 Link5_U;
    Scalar Link5_D;
    Scalar Link5_u;
    // Link 'Link6' :
    Matrix66 Link6_AI;
    Velocity Link6_a;
    Velocity Link6_v;
    Velocity Link6_c;
    Force    Link6_p;

    Column6 Link6_U;
    Scalar Link6_D;
    Scalar Link6_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_Link1_X_fr_base_link)(q);
    (motionTransforms-> fr_Link2_X_fr_Link1)(q);
    (motionTransforms-> fr_Link3_X_fr_Link2)(q);
    (motionTransforms-> fr_Link4_X_fr_Link3)(q);
    (motionTransforms-> fr_Link5_X_fr_Link4)(q);
    (motionTransforms-> fr_Link6_X_fr_Link5)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd, Acceleration& base_link_a, // output parameters,
    const Velocity& base_link_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, base_link_a, base_link_v, g, qd, tau, fext);
}

}
}

#endif
