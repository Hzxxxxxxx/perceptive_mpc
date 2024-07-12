#ifndef RCG_RM_65_EXAMPLE_INVERSE_DYNAMICS_H_
#define RCG_RM_65_EXAMPLE_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace rm_65_example {
namespace rcg {

/**
 * The Inverse Dynamics routine for the robot rm_65_example.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */
class InverseDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot rm_65_example, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[out] baseAccel the spatial acceleration of the robot base
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_link_v the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JointState& jForces, Acceleration& base_link_a,
        const Acceleration& g, const Velocity& base_link_v,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces, Acceleration& base_link_a,
        const Acceleration& g, const Velocity& base_link_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] baseWrench the spatial force to be applied to the robot base to achieve
     *             the desired accelerations
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_link_v the spatial velocity of the base
     * \param[in] baseAccel the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const JointState& q);
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g);
    ///@}
    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_link_v, const JointState& q, const JointState& qd);
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_link_v, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Force& getForce_base_link() const { return base_link_f; }
    const Velocity& getVelocity_Link1() const { return Link1_v; }
    const Acceleration& getAcceleration_Link1() const { return Link1_a; }
    const Force& getForce_Link1() const { return Link1_f; }
    const Velocity& getVelocity_Link2() const { return Link2_v; }
    const Acceleration& getAcceleration_Link2() const { return Link2_a; }
    const Force& getForce_Link2() const { return Link2_f; }
    const Velocity& getVelocity_Link3() const { return Link3_v; }
    const Acceleration& getAcceleration_Link3() const { return Link3_a; }
    const Force& getForce_Link3() const { return Link3_f; }
    const Velocity& getVelocity_Link4() const { return Link4_v; }
    const Acceleration& getAcceleration_Link4() const { return Link4_a; }
    const Force& getForce_Link4() const { return Link4_f; }
    const Velocity& getVelocity_Link5() const { return Link5_v; }
    const Acceleration& getAcceleration_Link5() const { return Link5_a; }
    const Force& getForce_Link5() const { return Link5_f; }
    const Velocity& getVelocity_Link6() const { return Link6_v; }
    const Acceleration& getAcceleration_Link6() const { return Link6_a; }
    const Force& getForce_Link6() const { return Link6_f; }
    ///@}
protected:
    void secondPass_fullyActuated(JointState& jForces);

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* xm;
private:
    Matrix66 vcross; // support variable
    // Link 'Link1' :
    const InertiaMatrix& Link1_I;
    Velocity      Link1_v;
    Acceleration  Link1_a;
    Force         Link1_f;
    // Link 'Link2' :
    const InertiaMatrix& Link2_I;
    Velocity      Link2_v;
    Acceleration  Link2_a;
    Force         Link2_f;
    // Link 'Link3' :
    const InertiaMatrix& Link3_I;
    Velocity      Link3_v;
    Acceleration  Link3_a;
    Force         Link3_f;
    // Link 'Link4' :
    const InertiaMatrix& Link4_I;
    Velocity      Link4_v;
    Acceleration  Link4_a;
    Force         Link4_f;
    // Link 'Link5' :
    const InertiaMatrix& Link5_I;
    Velocity      Link5_v;
    Acceleration  Link5_a;
    Force         Link5_f;
    // Link 'Link6' :
    const InertiaMatrix& Link6_I;
    Velocity      Link6_v;
    Acceleration  Link6_a;
    Force         Link6_f;

    // The robot base
    const InertiaMatrix& base_link_I;
    InertiaMatrix base_link_Ic;
    Force         base_link_f;
    // The composite inertia tensors
    InertiaMatrix Link1_Ic;
    InertiaMatrix Link2_Ic;
    InertiaMatrix Link3_Ic;
    InertiaMatrix Link4_Ic;
    InertiaMatrix Link5_Ic;
    const InertiaMatrix& Link6_Ic;

private:
    static const ExtForces zeroExtForces;
};

inline void InverseDynamics::setJointStatus(const JointState& q) const
{
    (xm->fr_Link1_X_fr_base_link)(q);
    (xm->fr_Link2_X_fr_Link1)(q);
    (xm->fr_Link3_X_fr_Link2)(q);
    (xm->fr_Link4_X_fr_Link3)(q);
    (xm->fr_Link5_X_fr_Link4)(q);
    (xm->fr_Link6_X_fr_Link5)(q);
}

inline void InverseDynamics::id(
    JointState& jForces, Acceleration& base_link_a,
    const Acceleration& g, const Velocity& base_link_v,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, base_link_a, g, base_link_v,
       qd, qdd, fext);
}

inline void InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g, const JointState& q)
{
    setJointStatus(q);
    G_terms_fully_actuated(baseWrench, jForces, g);
}

inline void InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_v, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms_fully_actuated(baseWrench, jForces, base_link_v, qd);
}

inline void InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    setJointStatus(q);
    id_fully_actuated(baseWrench, jForces, g, base_link_v,
        baseAccel, qd, qdd, fext);
}

}
}

#endif
