#ifndef RM_65_EXAMPLE_TRANSFORMS_H_
#define RM_65_EXAMPLE_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace rm_65_example {
namespace rcg {

struct Parameters
{
    struct AngleFuncValues {
        AngleFuncValues() {
            update();
        }

        void update()
        {
        }
    };

    Params_lengths lengths;
    Params_angles angles;
    AngleFuncValues trig = AngleFuncValues();
};

// The type of the "vector" with the status of the variables
typedef JointState state_t;

template<class M>
using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms
{
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;

    struct Type_fr_Link1_X_fr_base_link : public TransformMotion<Type_fr_Link1_X_fr_base_link>
    {
        Type_fr_Link1_X_fr_base_link();
        const Type_fr_Link1_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_Link1 : public TransformMotion<Type_fr_base_link_X_fr_Link1>
    {
        Type_fr_base_link_X_fr_Link1();
        const Type_fr_base_link_X_fr_Link1& update(const state_t&);
    };
    
    struct Type_fr_Link2_X_fr_Link1 : public TransformMotion<Type_fr_Link2_X_fr_Link1>
    {
        Type_fr_Link2_X_fr_Link1();
        const Type_fr_Link2_X_fr_Link1& update(const state_t&);
    };
    
    struct Type_fr_Link1_X_fr_Link2 : public TransformMotion<Type_fr_Link1_X_fr_Link2>
    {
        Type_fr_Link1_X_fr_Link2();
        const Type_fr_Link1_X_fr_Link2& update(const state_t&);
    };
    
    struct Type_fr_Link3_X_fr_Link2 : public TransformMotion<Type_fr_Link3_X_fr_Link2>
    {
        Type_fr_Link3_X_fr_Link2();
        const Type_fr_Link3_X_fr_Link2& update(const state_t&);
    };
    
    struct Type_fr_Link2_X_fr_Link3 : public TransformMotion<Type_fr_Link2_X_fr_Link3>
    {
        Type_fr_Link2_X_fr_Link3();
        const Type_fr_Link2_X_fr_Link3& update(const state_t&);
    };
    
    struct Type_fr_Link4_X_fr_Link3 : public TransformMotion<Type_fr_Link4_X_fr_Link3>
    {
        Type_fr_Link4_X_fr_Link3();
        const Type_fr_Link4_X_fr_Link3& update(const state_t&);
    };
    
    struct Type_fr_Link3_X_fr_Link4 : public TransformMotion<Type_fr_Link3_X_fr_Link4>
    {
        Type_fr_Link3_X_fr_Link4();
        const Type_fr_Link3_X_fr_Link4& update(const state_t&);
    };
    
    struct Type_fr_Link5_X_fr_Link4 : public TransformMotion<Type_fr_Link5_X_fr_Link4>
    {
        Type_fr_Link5_X_fr_Link4();
        const Type_fr_Link5_X_fr_Link4& update(const state_t&);
    };
    
    struct Type_fr_Link4_X_fr_Link5 : public TransformMotion<Type_fr_Link4_X_fr_Link5>
    {
        Type_fr_Link4_X_fr_Link5();
        const Type_fr_Link4_X_fr_Link5& update(const state_t&);
    };
    
    struct Type_fr_Link6_X_fr_Link5 : public TransformMotion<Type_fr_Link6_X_fr_Link5>
    {
        Type_fr_Link6_X_fr_Link5();
        const Type_fr_Link6_X_fr_Link5& update(const state_t&);
    };
    
    struct Type_fr_Link5_X_fr_Link6 : public TransformMotion<Type_fr_Link5_X_fr_Link6>
    {
        Type_fr_Link5_X_fr_Link6();
        const Type_fr_Link5_X_fr_Link6& update(const state_t&);
    };
    
public:
    MotionTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_Link1_X_fr_base_link fr_Link1_X_fr_base_link;
    Type_fr_base_link_X_fr_Link1 fr_base_link_X_fr_Link1;
    Type_fr_Link2_X_fr_Link1 fr_Link2_X_fr_Link1;
    Type_fr_Link1_X_fr_Link2 fr_Link1_X_fr_Link2;
    Type_fr_Link3_X_fr_Link2 fr_Link3_X_fr_Link2;
    Type_fr_Link2_X_fr_Link3 fr_Link2_X_fr_Link3;
    Type_fr_Link4_X_fr_Link3 fr_Link4_X_fr_Link3;
    Type_fr_Link3_X_fr_Link4 fr_Link3_X_fr_Link4;
    Type_fr_Link5_X_fr_Link4 fr_Link5_X_fr_Link4;
    Type_fr_Link4_X_fr_Link5 fr_Link4_X_fr_Link5;
    Type_fr_Link6_X_fr_Link5 fr_Link6_X_fr_Link5;
    Type_fr_Link5_X_fr_Link6 fr_Link5_X_fr_Link6;

protected:
    Parameters params;

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms
{
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;

    struct Type_fr_Link1_X_fr_base_link : public TransformForce<Type_fr_Link1_X_fr_base_link>
    {
        Type_fr_Link1_X_fr_base_link();
        const Type_fr_Link1_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_Link1 : public TransformForce<Type_fr_base_link_X_fr_Link1>
    {
        Type_fr_base_link_X_fr_Link1();
        const Type_fr_base_link_X_fr_Link1& update(const state_t&);
    };
    
    struct Type_fr_Link2_X_fr_Link1 : public TransformForce<Type_fr_Link2_X_fr_Link1>
    {
        Type_fr_Link2_X_fr_Link1();
        const Type_fr_Link2_X_fr_Link1& update(const state_t&);
    };
    
    struct Type_fr_Link1_X_fr_Link2 : public TransformForce<Type_fr_Link1_X_fr_Link2>
    {
        Type_fr_Link1_X_fr_Link2();
        const Type_fr_Link1_X_fr_Link2& update(const state_t&);
    };
    
    struct Type_fr_Link3_X_fr_Link2 : public TransformForce<Type_fr_Link3_X_fr_Link2>
    {
        Type_fr_Link3_X_fr_Link2();
        const Type_fr_Link3_X_fr_Link2& update(const state_t&);
    };
    
    struct Type_fr_Link2_X_fr_Link3 : public TransformForce<Type_fr_Link2_X_fr_Link3>
    {
        Type_fr_Link2_X_fr_Link3();
        const Type_fr_Link2_X_fr_Link3& update(const state_t&);
    };
    
    struct Type_fr_Link4_X_fr_Link3 : public TransformForce<Type_fr_Link4_X_fr_Link3>
    {
        Type_fr_Link4_X_fr_Link3();
        const Type_fr_Link4_X_fr_Link3& update(const state_t&);
    };
    
    struct Type_fr_Link3_X_fr_Link4 : public TransformForce<Type_fr_Link3_X_fr_Link4>
    {
        Type_fr_Link3_X_fr_Link4();
        const Type_fr_Link3_X_fr_Link4& update(const state_t&);
    };
    
    struct Type_fr_Link5_X_fr_Link4 : public TransformForce<Type_fr_Link5_X_fr_Link4>
    {
        Type_fr_Link5_X_fr_Link4();
        const Type_fr_Link5_X_fr_Link4& update(const state_t&);
    };
    
    struct Type_fr_Link4_X_fr_Link5 : public TransformForce<Type_fr_Link4_X_fr_Link5>
    {
        Type_fr_Link4_X_fr_Link5();
        const Type_fr_Link4_X_fr_Link5& update(const state_t&);
    };
    
    struct Type_fr_Link6_X_fr_Link5 : public TransformForce<Type_fr_Link6_X_fr_Link5>
    {
        Type_fr_Link6_X_fr_Link5();
        const Type_fr_Link6_X_fr_Link5& update(const state_t&);
    };
    
    struct Type_fr_Link5_X_fr_Link6 : public TransformForce<Type_fr_Link5_X_fr_Link6>
    {
        Type_fr_Link5_X_fr_Link6();
        const Type_fr_Link5_X_fr_Link6& update(const state_t&);
    };
    
public:
    ForceTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_Link1_X_fr_base_link fr_Link1_X_fr_base_link;
    Type_fr_base_link_X_fr_Link1 fr_base_link_X_fr_Link1;
    Type_fr_Link2_X_fr_Link1 fr_Link2_X_fr_Link1;
    Type_fr_Link1_X_fr_Link2 fr_Link1_X_fr_Link2;
    Type_fr_Link3_X_fr_Link2 fr_Link3_X_fr_Link2;
    Type_fr_Link2_X_fr_Link3 fr_Link2_X_fr_Link3;
    Type_fr_Link4_X_fr_Link3 fr_Link4_X_fr_Link3;
    Type_fr_Link3_X_fr_Link4 fr_Link3_X_fr_Link4;
    Type_fr_Link5_X_fr_Link4 fr_Link5_X_fr_Link4;
    Type_fr_Link4_X_fr_Link5 fr_Link4_X_fr_Link5;
    Type_fr_Link6_X_fr_Link5 fr_Link6_X_fr_Link5;
    Type_fr_Link5_X_fr_Link6 fr_Link5_X_fr_Link6;

protected:
    Parameters params;

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms
{
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

    struct Type_fr_Link1_X_fr_base_link : public TransformHomogeneous<Type_fr_Link1_X_fr_base_link>
    {
        Type_fr_Link1_X_fr_base_link();
        const Type_fr_Link1_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_Link1 : public TransformHomogeneous<Type_fr_base_link_X_fr_Link1>
    {
        Type_fr_base_link_X_fr_Link1();
        const Type_fr_base_link_X_fr_Link1& update(const state_t&);
    };
    
    struct Type_fr_Link2_X_fr_Link1 : public TransformHomogeneous<Type_fr_Link2_X_fr_Link1>
    {
        Type_fr_Link2_X_fr_Link1();
        const Type_fr_Link2_X_fr_Link1& update(const state_t&);
    };
    
    struct Type_fr_Link1_X_fr_Link2 : public TransformHomogeneous<Type_fr_Link1_X_fr_Link2>
    {
        Type_fr_Link1_X_fr_Link2();
        const Type_fr_Link1_X_fr_Link2& update(const state_t&);
    };
    
    struct Type_fr_Link3_X_fr_Link2 : public TransformHomogeneous<Type_fr_Link3_X_fr_Link2>
    {
        Type_fr_Link3_X_fr_Link2();
        const Type_fr_Link3_X_fr_Link2& update(const state_t&);
    };
    
    struct Type_fr_Link2_X_fr_Link3 : public TransformHomogeneous<Type_fr_Link2_X_fr_Link3>
    {
        Type_fr_Link2_X_fr_Link3();
        const Type_fr_Link2_X_fr_Link3& update(const state_t&);
    };
    
    struct Type_fr_Link4_X_fr_Link3 : public TransformHomogeneous<Type_fr_Link4_X_fr_Link3>
    {
        Type_fr_Link4_X_fr_Link3();
        const Type_fr_Link4_X_fr_Link3& update(const state_t&);
    };
    
    struct Type_fr_Link3_X_fr_Link4 : public TransformHomogeneous<Type_fr_Link3_X_fr_Link4>
    {
        Type_fr_Link3_X_fr_Link4();
        const Type_fr_Link3_X_fr_Link4& update(const state_t&);
    };
    
    struct Type_fr_Link5_X_fr_Link4 : public TransformHomogeneous<Type_fr_Link5_X_fr_Link4>
    {
        Type_fr_Link5_X_fr_Link4();
        const Type_fr_Link5_X_fr_Link4& update(const state_t&);
    };
    
    struct Type_fr_Link4_X_fr_Link5 : public TransformHomogeneous<Type_fr_Link4_X_fr_Link5>
    {
        Type_fr_Link4_X_fr_Link5();
        const Type_fr_Link4_X_fr_Link5& update(const state_t&);
    };
    
    struct Type_fr_Link6_X_fr_Link5 : public TransformHomogeneous<Type_fr_Link6_X_fr_Link5>
    {
        Type_fr_Link6_X_fr_Link5();
        const Type_fr_Link6_X_fr_Link5& update(const state_t&);
    };
    
    struct Type_fr_Link5_X_fr_Link6 : public TransformHomogeneous<Type_fr_Link5_X_fr_Link6>
    {
        Type_fr_Link5_X_fr_Link6();
        const Type_fr_Link5_X_fr_Link6& update(const state_t&);
    };
    
public:
    HomogeneousTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_Link1_X_fr_base_link fr_Link1_X_fr_base_link;
    Type_fr_base_link_X_fr_Link1 fr_base_link_X_fr_Link1;
    Type_fr_Link2_X_fr_Link1 fr_Link2_X_fr_Link1;
    Type_fr_Link1_X_fr_Link2 fr_Link1_X_fr_Link2;
    Type_fr_Link3_X_fr_Link2 fr_Link3_X_fr_Link2;
    Type_fr_Link2_X_fr_Link3 fr_Link2_X_fr_Link3;
    Type_fr_Link4_X_fr_Link3 fr_Link4_X_fr_Link3;
    Type_fr_Link3_X_fr_Link4 fr_Link3_X_fr_Link4;
    Type_fr_Link5_X_fr_Link4 fr_Link5_X_fr_Link4;
    Type_fr_Link4_X_fr_Link5 fr_Link4_X_fr_Link5;
    Type_fr_Link6_X_fr_Link5 fr_Link6_X_fr_Link5;
    Type_fr_Link5_X_fr_Link6 fr_Link5_X_fr_Link6;

protected:
    Parameters params;

}; //class 'HomogeneousTransforms'

}
}

#endif
