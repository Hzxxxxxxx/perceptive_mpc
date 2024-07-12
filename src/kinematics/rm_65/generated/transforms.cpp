#include "transforms.h"

using namespace rm_65_example::rcg;

// Constructors

MotionTransforms::MotionTransforms()
 :     fr_Link1_X_fr_base_link(),
    fr_base_link_X_fr_Link1(),
    fr_Link2_X_fr_Link1(),
    fr_Link1_X_fr_Link2(),
    fr_Link3_X_fr_Link2(),
    fr_Link2_X_fr_Link3(),
    fr_Link4_X_fr_Link3(),
    fr_Link3_X_fr_Link4(),
    fr_Link5_X_fr_Link4(),
    fr_Link4_X_fr_Link5(),
    fr_Link6_X_fr_Link5(),
    fr_Link5_X_fr_Link6()
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_Link1_X_fr_base_link(),
    fr_base_link_X_fr_Link1(),
    fr_Link2_X_fr_Link1(),
    fr_Link1_X_fr_Link2(),
    fr_Link3_X_fr_Link2(),
    fr_Link2_X_fr_Link3(),
    fr_Link4_X_fr_Link3(),
    fr_Link3_X_fr_Link4(),
    fr_Link5_X_fr_Link4(),
    fr_Link4_X_fr_Link5(),
    fr_Link6_X_fr_Link5(),
    fr_Link5_X_fr_Link6()
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_Link1_X_fr_base_link(),
    fr_base_link_X_fr_Link1(),
    fr_Link2_X_fr_Link1(),
    fr_Link1_X_fr_Link2(),
    fr_Link3_X_fr_Link2(),
    fr_Link2_X_fr_Link3(),
    fr_Link4_X_fr_Link3(),
    fr_Link3_X_fr_Link4(),
    fr_Link5_X_fr_Link4(),
    fr_Link4_X_fr_Link5(),
    fr_Link6_X_fr_Link5(),
    fr_Link5_X_fr_Link6()
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_Link1_X_fr_base_link::Type_fr_Link1_X_fr_base_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_Link1_X_fr_base_link& MotionTransforms::Type_fr_Link1_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_joint1  = ScalarTraits::sin( q(JOINT1) );
    Scalar cos_q_joint1  = ScalarTraits::cos( q(JOINT1) );
    (*this)(0,0) = cos_q_joint1;
    (*this)(0,1) = sin_q_joint1;
    (*this)(1,0) = -sin_q_joint1;
    (*this)(1,1) = cos_q_joint1;
    (*this)(3,0) = - tz_joint1 * sin_q_joint1;
    (*this)(3,1) =  tz_joint1 * cos_q_joint1;
    (*this)(3,3) = cos_q_joint1;
    (*this)(3,4) = sin_q_joint1;
    (*this)(4,0) = - tz_joint1 * cos_q_joint1;
    (*this)(4,1) = - tz_joint1 * sin_q_joint1;
    (*this)(4,3) = -sin_q_joint1;
    (*this)(4,4) = cos_q_joint1;
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_Link1::Type_fr_base_link_X_fr_Link1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_Link1& MotionTransforms::Type_fr_base_link_X_fr_Link1::update(const state_t& q)
{
    Scalar sin_q_joint1  = ScalarTraits::sin( q(JOINT1) );
    Scalar cos_q_joint1  = ScalarTraits::cos( q(JOINT1) );
    (*this)(0,0) = cos_q_joint1;
    (*this)(0,1) = -sin_q_joint1;
    (*this)(1,0) = sin_q_joint1;
    (*this)(1,1) = cos_q_joint1;
    (*this)(3,0) = - tz_joint1 * sin_q_joint1;
    (*this)(3,1) = - tz_joint1 * cos_q_joint1;
    (*this)(3,3) = cos_q_joint1;
    (*this)(3,4) = -sin_q_joint1;
    (*this)(4,0) =  tz_joint1 * cos_q_joint1;
    (*this)(4,1) = - tz_joint1 * sin_q_joint1;
    (*this)(4,3) = sin_q_joint1;
    (*this)(4,4) = cos_q_joint1;
    return *this;
}
MotionTransforms::Type_fr_Link2_X_fr_Link1::Type_fr_Link2_X_fr_Link1()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_Link2_X_fr_Link1& MotionTransforms::Type_fr_Link2_X_fr_Link1::update(const state_t& q)
{
    Scalar sin_q_joint2  = ScalarTraits::sin( q(JOINT2) );
    Scalar cos_q_joint2  = ScalarTraits::cos( q(JOINT2) );
    (*this)(0,0) = cos_q_joint2;
    (*this)(0,2) = sin_q_joint2;
    (*this)(1,0) = -sin_q_joint2;
    (*this)(1,2) = cos_q_joint2;
    (*this)(3,3) = cos_q_joint2;
    (*this)(3,5) = sin_q_joint2;
    (*this)(4,3) = -sin_q_joint2;
    (*this)(4,5) = cos_q_joint2;
    return *this;
}
MotionTransforms::Type_fr_Link1_X_fr_Link2::Type_fr_Link1_X_fr_Link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_Link1_X_fr_Link2& MotionTransforms::Type_fr_Link1_X_fr_Link2::update(const state_t& q)
{
    Scalar sin_q_joint2  = ScalarTraits::sin( q(JOINT2) );
    Scalar cos_q_joint2  = ScalarTraits::cos( q(JOINT2) );
    (*this)(0,0) = cos_q_joint2;
    (*this)(0,1) = -sin_q_joint2;
    (*this)(2,0) = sin_q_joint2;
    (*this)(2,1) = cos_q_joint2;
    (*this)(3,3) = cos_q_joint2;
    (*this)(3,4) = -sin_q_joint2;
    (*this)(5,3) = sin_q_joint2;
    (*this)(5,4) = cos_q_joint2;
    return *this;
}
MotionTransforms::Type_fr_Link3_X_fr_Link2::Type_fr_Link3_X_fr_Link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_joint3;    // Maxima DSL: _k__ty_joint3
    (*this)(5,1) = - tx_joint3;    // Maxima DSL: -_k__tx_joint3
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_Link3_X_fr_Link2& MotionTransforms::Type_fr_Link3_X_fr_Link2::update(const state_t& q)
{
    Scalar sin_q_joint3  = ScalarTraits::sin( q(JOINT3) );
    Scalar cos_q_joint3  = ScalarTraits::cos( q(JOINT3) );
    (*this)(0,0) = cos_q_joint3;
    (*this)(0,1) = sin_q_joint3;
    (*this)(1,0) = -sin_q_joint3;
    (*this)(1,1) = cos_q_joint3;
    (*this)(3,2) = ( tx_joint3 * sin_q_joint3)-( ty_joint3 * cos_q_joint3);
    (*this)(3,3) = cos_q_joint3;
    (*this)(3,4) = sin_q_joint3;
    (*this)(4,2) = ( ty_joint3 * sin_q_joint3)+( tx_joint3 * cos_q_joint3);
    (*this)(4,3) = -sin_q_joint3;
    (*this)(4,4) = cos_q_joint3;
    return *this;
}
MotionTransforms::Type_fr_Link2_X_fr_Link3::Type_fr_Link2_X_fr_Link3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) =  ty_joint3;    // Maxima DSL: _k__ty_joint3
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_joint3;    // Maxima DSL: -_k__tx_joint3
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_Link2_X_fr_Link3& MotionTransforms::Type_fr_Link2_X_fr_Link3::update(const state_t& q)
{
    Scalar sin_q_joint3  = ScalarTraits::sin( q(JOINT3) );
    Scalar cos_q_joint3  = ScalarTraits::cos( q(JOINT3) );
    (*this)(0,0) = cos_q_joint3;
    (*this)(0,1) = -sin_q_joint3;
    (*this)(1,0) = sin_q_joint3;
    (*this)(1,1) = cos_q_joint3;
    (*this)(3,3) = cos_q_joint3;
    (*this)(3,4) = -sin_q_joint3;
    (*this)(4,3) = sin_q_joint3;
    (*this)(4,4) = cos_q_joint3;
    (*this)(5,0) = ( tx_joint3 * sin_q_joint3)-( ty_joint3 * cos_q_joint3);
    (*this)(5,1) = ( ty_joint3 * sin_q_joint3)+( tx_joint3 * cos_q_joint3);
    return *this;
}
MotionTransforms::Type_fr_Link4_X_fr_Link3::Type_fr_Link4_X_fr_Link3()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(2,2) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) =  ty_joint4 * cos_rx_joint4;    // Maxima DSL: _k__ty_joint4*cos(_k__rx_joint4)
    (*this)(5,1) = - tx_joint4 * cos_rx_joint4;    // Maxima DSL: -_k__tx_joint4*cos(_k__rx_joint4)
    (*this)(5,2) = - tx_joint4 * sin_rx_joint4;    // Maxima DSL: -_k__tx_joint4*sin(_k__rx_joint4)
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(5,5) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
}

const MotionTransforms::Type_fr_Link4_X_fr_Link3& MotionTransforms::Type_fr_Link4_X_fr_Link3::update(const state_t& q)
{
    Scalar sin_q_joint4  = ScalarTraits::sin( q(JOINT4) );
    Scalar cos_q_joint4  = ScalarTraits::cos( q(JOINT4) );
    (*this)(0,0) = cos_q_joint4;
    (*this)(0,1) = cos_rx_joint4 * sin_q_joint4;
    (*this)(0,2) = sin_rx_joint4 * sin_q_joint4;
    (*this)(1,0) = -sin_q_joint4;
    (*this)(1,1) = cos_rx_joint4 * cos_q_joint4;
    (*this)(1,2) = sin_rx_joint4 * cos_q_joint4;
    (*this)(3,0) =  ty_joint4 * sin_rx_joint4 * sin_q_joint4;
    (*this)(3,1) = - tx_joint4 * sin_rx_joint4 * sin_q_joint4;
    (*this)(3,2) = ( tx_joint4 * cos_rx_joint4 * sin_q_joint4)-( ty_joint4 * cos_q_joint4);
    (*this)(3,3) = cos_q_joint4;
    (*this)(3,4) = cos_rx_joint4 * sin_q_joint4;
    (*this)(3,5) = sin_rx_joint4 * sin_q_joint4;
    (*this)(4,0) =  ty_joint4 * sin_rx_joint4 * cos_q_joint4;
    (*this)(4,1) = - tx_joint4 * sin_rx_joint4 * cos_q_joint4;
    (*this)(4,2) = ( ty_joint4 * sin_q_joint4)+( tx_joint4 * cos_rx_joint4 * cos_q_joint4);
    (*this)(4,3) = -sin_q_joint4;
    (*this)(4,4) = cos_rx_joint4 * cos_q_joint4;
    (*this)(4,5) = sin_rx_joint4 * cos_q_joint4;
    return *this;
}
MotionTransforms::Type_fr_Link3_X_fr_Link4::Type_fr_Link3_X_fr_Link4()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) =  ty_joint4 * cos_rx_joint4;    // Maxima DSL: _k__ty_joint4*cos(_k__rx_joint4)
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_joint4 * cos_rx_joint4;    // Maxima DSL: -_k__tx_joint4*cos(_k__rx_joint4)
    (*this)(4,5) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(5,2) = - tx_joint4 * sin_rx_joint4;    // Maxima DSL: -_k__tx_joint4*sin(_k__rx_joint4)
    (*this)(5,5) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
}

const MotionTransforms::Type_fr_Link3_X_fr_Link4& MotionTransforms::Type_fr_Link3_X_fr_Link4::update(const state_t& q)
{
    Scalar sin_q_joint4  = ScalarTraits::sin( q(JOINT4) );
    Scalar cos_q_joint4  = ScalarTraits::cos( q(JOINT4) );
    (*this)(0,0) = cos_q_joint4;
    (*this)(0,1) = -sin_q_joint4;
    (*this)(1,0) = cos_rx_joint4 * sin_q_joint4;
    (*this)(1,1) = cos_rx_joint4 * cos_q_joint4;
    (*this)(2,0) = sin_rx_joint4 * sin_q_joint4;
    (*this)(2,1) = sin_rx_joint4 * cos_q_joint4;
    (*this)(3,0) =  ty_joint4 * sin_rx_joint4 * sin_q_joint4;
    (*this)(3,1) =  ty_joint4 * sin_rx_joint4 * cos_q_joint4;
    (*this)(3,3) = cos_q_joint4;
    (*this)(3,4) = -sin_q_joint4;
    (*this)(4,0) = - tx_joint4 * sin_rx_joint4 * sin_q_joint4;
    (*this)(4,1) = - tx_joint4 * sin_rx_joint4 * cos_q_joint4;
    (*this)(4,3) = cos_rx_joint4 * sin_q_joint4;
    (*this)(4,4) = cos_rx_joint4 * cos_q_joint4;
    (*this)(5,0) = ( tx_joint4 * cos_rx_joint4 * sin_q_joint4)-( ty_joint4 * cos_q_joint4);
    (*this)(5,1) = ( ty_joint4 * sin_q_joint4)+( tx_joint4 * cos_rx_joint4 * cos_q_joint4);
    (*this)(5,3) = sin_rx_joint4 * sin_q_joint4;
    (*this)(5,4) = sin_rx_joint4 * cos_q_joint4;
    return *this;
}
MotionTransforms::Type_fr_Link5_X_fr_Link4::Type_fr_Link5_X_fr_Link4()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(2,2) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(5,5) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
}

const MotionTransforms::Type_fr_Link5_X_fr_Link4& MotionTransforms::Type_fr_Link5_X_fr_Link4::update(const state_t& q)
{
    Scalar sin_q_joint5  = ScalarTraits::sin( q(JOINT5) );
    Scalar cos_q_joint5  = ScalarTraits::cos( q(JOINT5) );
    (*this)(0,0) = cos_q_joint5;
    (*this)(0,1) = cos_rx_joint5 * sin_q_joint5;
    (*this)(0,2) = sin_rx_joint5 * sin_q_joint5;
    (*this)(1,0) = -sin_q_joint5;
    (*this)(1,1) = cos_rx_joint5 * cos_q_joint5;
    (*this)(1,2) = sin_rx_joint5 * cos_q_joint5;
    (*this)(3,3) = cos_q_joint5;
    (*this)(3,4) = cos_rx_joint5 * sin_q_joint5;
    (*this)(3,5) = sin_rx_joint5 * sin_q_joint5;
    (*this)(4,3) = -sin_q_joint5;
    (*this)(4,4) = cos_rx_joint5 * cos_q_joint5;
    (*this)(4,5) = sin_rx_joint5 * cos_q_joint5;
    return *this;
}
MotionTransforms::Type_fr_Link4_X_fr_Link5::Type_fr_Link4_X_fr_Link5()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
}

const MotionTransforms::Type_fr_Link4_X_fr_Link5& MotionTransforms::Type_fr_Link4_X_fr_Link5::update(const state_t& q)
{
    Scalar sin_q_joint5  = ScalarTraits::sin( q(JOINT5) );
    Scalar cos_q_joint5  = ScalarTraits::cos( q(JOINT5) );
    (*this)(0,0) = cos_q_joint5;
    (*this)(0,1) = -sin_q_joint5;
    (*this)(1,0) = cos_rx_joint5 * sin_q_joint5;
    (*this)(1,1) = cos_rx_joint5 * cos_q_joint5;
    (*this)(2,0) = sin_rx_joint5 * sin_q_joint5;
    (*this)(2,1) = sin_rx_joint5 * cos_q_joint5;
    (*this)(3,3) = cos_q_joint5;
    (*this)(3,4) = -sin_q_joint5;
    (*this)(4,3) = cos_rx_joint5 * sin_q_joint5;
    (*this)(4,4) = cos_rx_joint5 * cos_q_joint5;
    (*this)(5,3) = sin_rx_joint5 * sin_q_joint5;
    (*this)(5,4) = sin_rx_joint5 * cos_q_joint5;
    return *this;
}
MotionTransforms::Type_fr_Link6_X_fr_Link5::Type_fr_Link6_X_fr_Link5()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(2,2) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) =  ty_joint6 * cos_rx_joint6;    // Maxima DSL: _k__ty_joint6*cos(_k__rx_joint6)
    (*this)(5,1) = - tx_joint6 * cos_rx_joint6;    // Maxima DSL: -_k__tx_joint6*cos(_k__rx_joint6)
    (*this)(5,2) = - tx_joint6 * sin_rx_joint6;    // Maxima DSL: -_k__tx_joint6*sin(_k__rx_joint6)
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(5,5) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
}

const MotionTransforms::Type_fr_Link6_X_fr_Link5& MotionTransforms::Type_fr_Link6_X_fr_Link5::update(const state_t& q)
{
    Scalar sin_q_joint6  = ScalarTraits::sin( q(JOINT6) );
    Scalar cos_q_joint6  = ScalarTraits::cos( q(JOINT6) );
    (*this)(0,0) = cos_q_joint6;
    (*this)(0,1) = cos_rx_joint6 * sin_q_joint6;
    (*this)(0,2) = sin_rx_joint6 * sin_q_joint6;
    (*this)(1,0) = -sin_q_joint6;
    (*this)(1,1) = cos_rx_joint6 * cos_q_joint6;
    (*this)(1,2) = sin_rx_joint6 * cos_q_joint6;
    (*this)(3,0) =  ty_joint6 * sin_rx_joint6 * sin_q_joint6;
    (*this)(3,1) = - tx_joint6 * sin_rx_joint6 * sin_q_joint6;
    (*this)(3,2) = ( tx_joint6 * cos_rx_joint6 * sin_q_joint6)-( ty_joint6 * cos_q_joint6);
    (*this)(3,3) = cos_q_joint6;
    (*this)(3,4) = cos_rx_joint6 * sin_q_joint6;
    (*this)(3,5) = sin_rx_joint6 * sin_q_joint6;
    (*this)(4,0) =  ty_joint6 * sin_rx_joint6 * cos_q_joint6;
    (*this)(4,1) = - tx_joint6 * sin_rx_joint6 * cos_q_joint6;
    (*this)(4,2) = ( ty_joint6 * sin_q_joint6)+( tx_joint6 * cos_rx_joint6 * cos_q_joint6);
    (*this)(4,3) = -sin_q_joint6;
    (*this)(4,4) = cos_rx_joint6 * cos_q_joint6;
    (*this)(4,5) = sin_rx_joint6 * cos_q_joint6;
    return *this;
}
MotionTransforms::Type_fr_Link5_X_fr_Link6::Type_fr_Link5_X_fr_Link6()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) =  ty_joint6 * cos_rx_joint6;    // Maxima DSL: _k__ty_joint6*cos(_k__rx_joint6)
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_joint6 * cos_rx_joint6;    // Maxima DSL: -_k__tx_joint6*cos(_k__rx_joint6)
    (*this)(4,5) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(5,2) = - tx_joint6 * sin_rx_joint6;    // Maxima DSL: -_k__tx_joint6*sin(_k__rx_joint6)
    (*this)(5,5) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
}

const MotionTransforms::Type_fr_Link5_X_fr_Link6& MotionTransforms::Type_fr_Link5_X_fr_Link6::update(const state_t& q)
{
    Scalar sin_q_joint6  = ScalarTraits::sin( q(JOINT6) );
    Scalar cos_q_joint6  = ScalarTraits::cos( q(JOINT6) );
    (*this)(0,0) = cos_q_joint6;
    (*this)(0,1) = -sin_q_joint6;
    (*this)(1,0) = cos_rx_joint6 * sin_q_joint6;
    (*this)(1,1) = cos_rx_joint6 * cos_q_joint6;
    (*this)(2,0) = sin_rx_joint6 * sin_q_joint6;
    (*this)(2,1) = sin_rx_joint6 * cos_q_joint6;
    (*this)(3,0) =  ty_joint6 * sin_rx_joint6 * sin_q_joint6;
    (*this)(3,1) =  ty_joint6 * sin_rx_joint6 * cos_q_joint6;
    (*this)(3,3) = cos_q_joint6;
    (*this)(3,4) = -sin_q_joint6;
    (*this)(4,0) = - tx_joint6 * sin_rx_joint6 * sin_q_joint6;
    (*this)(4,1) = - tx_joint6 * sin_rx_joint6 * cos_q_joint6;
    (*this)(4,3) = cos_rx_joint6 * sin_q_joint6;
    (*this)(4,4) = cos_rx_joint6 * cos_q_joint6;
    (*this)(5,0) = ( tx_joint6 * cos_rx_joint6 * sin_q_joint6)-( ty_joint6 * cos_q_joint6);
    (*this)(5,1) = ( ty_joint6 * sin_q_joint6)+( tx_joint6 * cos_rx_joint6 * cos_q_joint6);
    (*this)(5,3) = sin_rx_joint6 * sin_q_joint6;
    (*this)(5,4) = sin_rx_joint6 * cos_q_joint6;
    return *this;
}

ForceTransforms::Type_fr_Link1_X_fr_base_link::Type_fr_Link1_X_fr_base_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_Link1_X_fr_base_link& ForceTransforms::Type_fr_Link1_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_joint1  = ScalarTraits::sin( q(JOINT1) );
    Scalar cos_q_joint1  = ScalarTraits::cos( q(JOINT1) );
    (*this)(0,0) = cos_q_joint1;
    (*this)(0,1) = sin_q_joint1;
    (*this)(0,3) = - tz_joint1 * sin_q_joint1;
    (*this)(0,4) =  tz_joint1 * cos_q_joint1;
    (*this)(1,0) = -sin_q_joint1;
    (*this)(1,1) = cos_q_joint1;
    (*this)(1,3) = - tz_joint1 * cos_q_joint1;
    (*this)(1,4) = - tz_joint1 * sin_q_joint1;
    (*this)(3,3) = cos_q_joint1;
    (*this)(3,4) = sin_q_joint1;
    (*this)(4,3) = -sin_q_joint1;
    (*this)(4,4) = cos_q_joint1;
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_Link1::Type_fr_base_link_X_fr_Link1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_Link1& ForceTransforms::Type_fr_base_link_X_fr_Link1::update(const state_t& q)
{
    Scalar sin_q_joint1  = ScalarTraits::sin( q(JOINT1) );
    Scalar cos_q_joint1  = ScalarTraits::cos( q(JOINT1) );
    (*this)(0,0) = cos_q_joint1;
    (*this)(0,1) = -sin_q_joint1;
    (*this)(0,3) = - tz_joint1 * sin_q_joint1;
    (*this)(0,4) = - tz_joint1 * cos_q_joint1;
    (*this)(1,0) = sin_q_joint1;
    (*this)(1,1) = cos_q_joint1;
    (*this)(1,3) =  tz_joint1 * cos_q_joint1;
    (*this)(1,4) = - tz_joint1 * sin_q_joint1;
    (*this)(3,3) = cos_q_joint1;
    (*this)(3,4) = -sin_q_joint1;
    (*this)(4,3) = sin_q_joint1;
    (*this)(4,4) = cos_q_joint1;
    return *this;
}
ForceTransforms::Type_fr_Link2_X_fr_Link1::Type_fr_Link2_X_fr_Link1()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_Link2_X_fr_Link1& ForceTransforms::Type_fr_Link2_X_fr_Link1::update(const state_t& q)
{
    Scalar sin_q_joint2  = ScalarTraits::sin( q(JOINT2) );
    Scalar cos_q_joint2  = ScalarTraits::cos( q(JOINT2) );
    (*this)(0,0) = cos_q_joint2;
    (*this)(0,2) = sin_q_joint2;
    (*this)(1,0) = -sin_q_joint2;
    (*this)(1,2) = cos_q_joint2;
    (*this)(3,3) = cos_q_joint2;
    (*this)(3,5) = sin_q_joint2;
    (*this)(4,3) = -sin_q_joint2;
    (*this)(4,5) = cos_q_joint2;
    return *this;
}
ForceTransforms::Type_fr_Link1_X_fr_Link2::Type_fr_Link1_X_fr_Link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = -1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_Link1_X_fr_Link2& ForceTransforms::Type_fr_Link1_X_fr_Link2::update(const state_t& q)
{
    Scalar sin_q_joint2  = ScalarTraits::sin( q(JOINT2) );
    Scalar cos_q_joint2  = ScalarTraits::cos( q(JOINT2) );
    (*this)(0,0) = cos_q_joint2;
    (*this)(0,1) = -sin_q_joint2;
    (*this)(2,0) = sin_q_joint2;
    (*this)(2,1) = cos_q_joint2;
    (*this)(3,3) = cos_q_joint2;
    (*this)(3,4) = -sin_q_joint2;
    (*this)(5,3) = sin_q_joint2;
    (*this)(5,4) = cos_q_joint2;
    return *this;
}
ForceTransforms::Type_fr_Link3_X_fr_Link2::Type_fr_Link3_X_fr_Link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_joint3;    // Maxima DSL: _k__ty_joint3
    (*this)(2,4) = - tx_joint3;    // Maxima DSL: -_k__tx_joint3
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_Link3_X_fr_Link2& ForceTransforms::Type_fr_Link3_X_fr_Link2::update(const state_t& q)
{
    Scalar sin_q_joint3  = ScalarTraits::sin( q(JOINT3) );
    Scalar cos_q_joint3  = ScalarTraits::cos( q(JOINT3) );
    (*this)(0,0) = cos_q_joint3;
    (*this)(0,1) = sin_q_joint3;
    (*this)(0,5) = ( tx_joint3 * sin_q_joint3)-( ty_joint3 * cos_q_joint3);
    (*this)(1,0) = -sin_q_joint3;
    (*this)(1,1) = cos_q_joint3;
    (*this)(1,5) = ( ty_joint3 * sin_q_joint3)+( tx_joint3 * cos_q_joint3);
    (*this)(3,3) = cos_q_joint3;
    (*this)(3,4) = sin_q_joint3;
    (*this)(4,3) = -sin_q_joint3;
    (*this)(4,4) = cos_q_joint3;
    return *this;
}
ForceTransforms::Type_fr_Link2_X_fr_Link3::Type_fr_Link2_X_fr_Link3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) =  ty_joint3;    // Maxima DSL: _k__ty_joint3
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_joint3;    // Maxima DSL: -_k__tx_joint3
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_Link2_X_fr_Link3& ForceTransforms::Type_fr_Link2_X_fr_Link3::update(const state_t& q)
{
    Scalar sin_q_joint3  = ScalarTraits::sin( q(JOINT3) );
    Scalar cos_q_joint3  = ScalarTraits::cos( q(JOINT3) );
    (*this)(0,0) = cos_q_joint3;
    (*this)(0,1) = -sin_q_joint3;
    (*this)(1,0) = sin_q_joint3;
    (*this)(1,1) = cos_q_joint3;
    (*this)(2,3) = ( tx_joint3 * sin_q_joint3)-( ty_joint3 * cos_q_joint3);
    (*this)(2,4) = ( ty_joint3 * sin_q_joint3)+( tx_joint3 * cos_q_joint3);
    (*this)(3,3) = cos_q_joint3;
    (*this)(3,4) = -sin_q_joint3;
    (*this)(4,3) = sin_q_joint3;
    (*this)(4,4) = cos_q_joint3;
    return *this;
}
ForceTransforms::Type_fr_Link4_X_fr_Link3::Type_fr_Link4_X_fr_Link3()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(2,2) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
    (*this)(2,3) =  ty_joint4 * cos_rx_joint4;    // Maxima DSL: _k__ty_joint4*cos(_k__rx_joint4)
    (*this)(2,4) = - tx_joint4 * cos_rx_joint4;    // Maxima DSL: -_k__tx_joint4*cos(_k__rx_joint4)
    (*this)(2,5) = - tx_joint4 * sin_rx_joint4;    // Maxima DSL: -_k__tx_joint4*sin(_k__rx_joint4)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(5,5) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
}

const ForceTransforms::Type_fr_Link4_X_fr_Link3& ForceTransforms::Type_fr_Link4_X_fr_Link3::update(const state_t& q)
{
    Scalar sin_q_joint4  = ScalarTraits::sin( q(JOINT4) );
    Scalar cos_q_joint4  = ScalarTraits::cos( q(JOINT4) );
    (*this)(0,0) = cos_q_joint4;
    (*this)(0,1) = cos_rx_joint4 * sin_q_joint4;
    (*this)(0,2) = sin_rx_joint4 * sin_q_joint4;
    (*this)(0,3) =  ty_joint4 * sin_rx_joint4 * sin_q_joint4;
    (*this)(0,4) = - tx_joint4 * sin_rx_joint4 * sin_q_joint4;
    (*this)(0,5) = ( tx_joint4 * cos_rx_joint4 * sin_q_joint4)-( ty_joint4 * cos_q_joint4);
    (*this)(1,0) = -sin_q_joint4;
    (*this)(1,1) = cos_rx_joint4 * cos_q_joint4;
    (*this)(1,2) = sin_rx_joint4 * cos_q_joint4;
    (*this)(1,3) =  ty_joint4 * sin_rx_joint4 * cos_q_joint4;
    (*this)(1,4) = - tx_joint4 * sin_rx_joint4 * cos_q_joint4;
    (*this)(1,5) = ( ty_joint4 * sin_q_joint4)+( tx_joint4 * cos_rx_joint4 * cos_q_joint4);
    (*this)(3,3) = cos_q_joint4;
    (*this)(3,4) = cos_rx_joint4 * sin_q_joint4;
    (*this)(3,5) = sin_rx_joint4 * sin_q_joint4;
    (*this)(4,3) = -sin_q_joint4;
    (*this)(4,4) = cos_rx_joint4 * cos_q_joint4;
    (*this)(4,5) = sin_rx_joint4 * cos_q_joint4;
    return *this;
}
ForceTransforms::Type_fr_Link3_X_fr_Link4::Type_fr_Link3_X_fr_Link4()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) =  ty_joint4 * cos_rx_joint4;    // Maxima DSL: _k__ty_joint4*cos(_k__rx_joint4)
    (*this)(1,2) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(1,5) = - tx_joint4 * cos_rx_joint4;    // Maxima DSL: -_k__tx_joint4*cos(_k__rx_joint4)
    (*this)(2,2) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
    (*this)(2,5) = - tx_joint4 * sin_rx_joint4;    // Maxima DSL: -_k__tx_joint4*sin(_k__rx_joint4)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
}

const ForceTransforms::Type_fr_Link3_X_fr_Link4& ForceTransforms::Type_fr_Link3_X_fr_Link4::update(const state_t& q)
{
    Scalar sin_q_joint4  = ScalarTraits::sin( q(JOINT4) );
    Scalar cos_q_joint4  = ScalarTraits::cos( q(JOINT4) );
    (*this)(0,0) = cos_q_joint4;
    (*this)(0,1) = -sin_q_joint4;
    (*this)(0,3) =  ty_joint4 * sin_rx_joint4 * sin_q_joint4;
    (*this)(0,4) =  ty_joint4 * sin_rx_joint4 * cos_q_joint4;
    (*this)(1,0) = cos_rx_joint4 * sin_q_joint4;
    (*this)(1,1) = cos_rx_joint4 * cos_q_joint4;
    (*this)(1,3) = - tx_joint4 * sin_rx_joint4 * sin_q_joint4;
    (*this)(1,4) = - tx_joint4 * sin_rx_joint4 * cos_q_joint4;
    (*this)(2,0) = sin_rx_joint4 * sin_q_joint4;
    (*this)(2,1) = sin_rx_joint4 * cos_q_joint4;
    (*this)(2,3) = ( tx_joint4 * cos_rx_joint4 * sin_q_joint4)-( ty_joint4 * cos_q_joint4);
    (*this)(2,4) = ( ty_joint4 * sin_q_joint4)+( tx_joint4 * cos_rx_joint4 * cos_q_joint4);
    (*this)(3,3) = cos_q_joint4;
    (*this)(3,4) = -sin_q_joint4;
    (*this)(4,3) = cos_rx_joint4 * sin_q_joint4;
    (*this)(4,4) = cos_rx_joint4 * cos_q_joint4;
    (*this)(5,3) = sin_rx_joint4 * sin_q_joint4;
    (*this)(5,4) = sin_rx_joint4 * cos_q_joint4;
    return *this;
}
ForceTransforms::Type_fr_Link5_X_fr_Link4::Type_fr_Link5_X_fr_Link4()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(2,2) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(5,5) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
}

const ForceTransforms::Type_fr_Link5_X_fr_Link4& ForceTransforms::Type_fr_Link5_X_fr_Link4::update(const state_t& q)
{
    Scalar sin_q_joint5  = ScalarTraits::sin( q(JOINT5) );
    Scalar cos_q_joint5  = ScalarTraits::cos( q(JOINT5) );
    (*this)(0,0) = cos_q_joint5;
    (*this)(0,1) = cos_rx_joint5 * sin_q_joint5;
    (*this)(0,2) = sin_rx_joint5 * sin_q_joint5;
    (*this)(1,0) = -sin_q_joint5;
    (*this)(1,1) = cos_rx_joint5 * cos_q_joint5;
    (*this)(1,2) = sin_rx_joint5 * cos_q_joint5;
    (*this)(3,3) = cos_q_joint5;
    (*this)(3,4) = cos_rx_joint5 * sin_q_joint5;
    (*this)(3,5) = sin_rx_joint5 * sin_q_joint5;
    (*this)(4,3) = -sin_q_joint5;
    (*this)(4,4) = cos_rx_joint5 * cos_q_joint5;
    (*this)(4,5) = sin_rx_joint5 * cos_q_joint5;
    return *this;
}
ForceTransforms::Type_fr_Link4_X_fr_Link5::Type_fr_Link4_X_fr_Link5()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
}

const ForceTransforms::Type_fr_Link4_X_fr_Link5& ForceTransforms::Type_fr_Link4_X_fr_Link5::update(const state_t& q)
{
    Scalar sin_q_joint5  = ScalarTraits::sin( q(JOINT5) );
    Scalar cos_q_joint5  = ScalarTraits::cos( q(JOINT5) );
    (*this)(0,0) = cos_q_joint5;
    (*this)(0,1) = -sin_q_joint5;
    (*this)(1,0) = cos_rx_joint5 * sin_q_joint5;
    (*this)(1,1) = cos_rx_joint5 * cos_q_joint5;
    (*this)(2,0) = sin_rx_joint5 * sin_q_joint5;
    (*this)(2,1) = sin_rx_joint5 * cos_q_joint5;
    (*this)(3,3) = cos_q_joint5;
    (*this)(3,4) = -sin_q_joint5;
    (*this)(4,3) = cos_rx_joint5 * sin_q_joint5;
    (*this)(4,4) = cos_rx_joint5 * cos_q_joint5;
    (*this)(5,3) = sin_rx_joint5 * sin_q_joint5;
    (*this)(5,4) = sin_rx_joint5 * cos_q_joint5;
    return *this;
}
ForceTransforms::Type_fr_Link6_X_fr_Link5::Type_fr_Link6_X_fr_Link5()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(2,2) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
    (*this)(2,3) =  ty_joint6 * cos_rx_joint6;    // Maxima DSL: _k__ty_joint6*cos(_k__rx_joint6)
    (*this)(2,4) = - tx_joint6 * cos_rx_joint6;    // Maxima DSL: -_k__tx_joint6*cos(_k__rx_joint6)
    (*this)(2,5) = - tx_joint6 * sin_rx_joint6;    // Maxima DSL: -_k__tx_joint6*sin(_k__rx_joint6)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(5,5) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
}

const ForceTransforms::Type_fr_Link6_X_fr_Link5& ForceTransforms::Type_fr_Link6_X_fr_Link5::update(const state_t& q)
{
    Scalar sin_q_joint6  = ScalarTraits::sin( q(JOINT6) );
    Scalar cos_q_joint6  = ScalarTraits::cos( q(JOINT6) );
    (*this)(0,0) = cos_q_joint6;
    (*this)(0,1) = cos_rx_joint6 * sin_q_joint6;
    (*this)(0,2) = sin_rx_joint6 * sin_q_joint6;
    (*this)(0,3) =  ty_joint6 * sin_rx_joint6 * sin_q_joint6;
    (*this)(0,4) = - tx_joint6 * sin_rx_joint6 * sin_q_joint6;
    (*this)(0,5) = ( tx_joint6 * cos_rx_joint6 * sin_q_joint6)-( ty_joint6 * cos_q_joint6);
    (*this)(1,0) = -sin_q_joint6;
    (*this)(1,1) = cos_rx_joint6 * cos_q_joint6;
    (*this)(1,2) = sin_rx_joint6 * cos_q_joint6;
    (*this)(1,3) =  ty_joint6 * sin_rx_joint6 * cos_q_joint6;
    (*this)(1,4) = - tx_joint6 * sin_rx_joint6 * cos_q_joint6;
    (*this)(1,5) = ( ty_joint6 * sin_q_joint6)+( tx_joint6 * cos_rx_joint6 * cos_q_joint6);
    (*this)(3,3) = cos_q_joint6;
    (*this)(3,4) = cos_rx_joint6 * sin_q_joint6;
    (*this)(3,5) = sin_rx_joint6 * sin_q_joint6;
    (*this)(4,3) = -sin_q_joint6;
    (*this)(4,4) = cos_rx_joint6 * cos_q_joint6;
    (*this)(4,5) = sin_rx_joint6 * cos_q_joint6;
    return *this;
}
ForceTransforms::Type_fr_Link5_X_fr_Link6::Type_fr_Link5_X_fr_Link6()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) =  ty_joint6 * cos_rx_joint6;    // Maxima DSL: _k__ty_joint6*cos(_k__rx_joint6)
    (*this)(1,2) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(1,5) = - tx_joint6 * cos_rx_joint6;    // Maxima DSL: -_k__tx_joint6*cos(_k__rx_joint6)
    (*this)(2,2) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
    (*this)(2,5) = - tx_joint6 * sin_rx_joint6;    // Maxima DSL: -_k__tx_joint6*sin(_k__rx_joint6)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
}

const ForceTransforms::Type_fr_Link5_X_fr_Link6& ForceTransforms::Type_fr_Link5_X_fr_Link6::update(const state_t& q)
{
    Scalar sin_q_joint6  = ScalarTraits::sin( q(JOINT6) );
    Scalar cos_q_joint6  = ScalarTraits::cos( q(JOINT6) );
    (*this)(0,0) = cos_q_joint6;
    (*this)(0,1) = -sin_q_joint6;
    (*this)(0,3) =  ty_joint6 * sin_rx_joint6 * sin_q_joint6;
    (*this)(0,4) =  ty_joint6 * sin_rx_joint6 * cos_q_joint6;
    (*this)(1,0) = cos_rx_joint6 * sin_q_joint6;
    (*this)(1,1) = cos_rx_joint6 * cos_q_joint6;
    (*this)(1,3) = - tx_joint6 * sin_rx_joint6 * sin_q_joint6;
    (*this)(1,4) = - tx_joint6 * sin_rx_joint6 * cos_q_joint6;
    (*this)(2,0) = sin_rx_joint6 * sin_q_joint6;
    (*this)(2,1) = sin_rx_joint6 * cos_q_joint6;
    (*this)(2,3) = ( tx_joint6 * cos_rx_joint6 * sin_q_joint6)-( ty_joint6 * cos_q_joint6);
    (*this)(2,4) = ( ty_joint6 * sin_q_joint6)+( tx_joint6 * cos_rx_joint6 * cos_q_joint6);
    (*this)(3,3) = cos_q_joint6;
    (*this)(3,4) = -sin_q_joint6;
    (*this)(4,3) = cos_rx_joint6 * sin_q_joint6;
    (*this)(4,4) = cos_rx_joint6 * cos_q_joint6;
    (*this)(5,3) = sin_rx_joint6 * sin_q_joint6;
    (*this)(5,4) = sin_rx_joint6 * cos_q_joint6;
    return *this;
}

HomogeneousTransforms::Type_fr_Link1_X_fr_base_link::Type_fr_Link1_X_fr_base_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_joint1;    // Maxima DSL: -_k__tz_joint1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link1_X_fr_base_link& HomogeneousTransforms::Type_fr_Link1_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_joint1  = ScalarTraits::sin( q(JOINT1) );
    Scalar cos_q_joint1  = ScalarTraits::cos( q(JOINT1) );
    (*this)(0,0) = cos_q_joint1;
    (*this)(0,1) = sin_q_joint1;
    (*this)(1,0) = -sin_q_joint1;
    (*this)(1,1) = cos_q_joint1;
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_Link1::Type_fr_base_link_X_fr_Link1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_joint1;    // Maxima DSL: _k__tz_joint1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_Link1& HomogeneousTransforms::Type_fr_base_link_X_fr_Link1::update(const state_t& q)
{
    Scalar sin_q_joint1  = ScalarTraits::sin( q(JOINT1) );
    Scalar cos_q_joint1  = ScalarTraits::cos( q(JOINT1) );
    (*this)(0,0) = cos_q_joint1;
    (*this)(0,1) = -sin_q_joint1;
    (*this)(1,0) = sin_q_joint1;
    (*this)(1,1) = cos_q_joint1;
    return *this;
}
HomogeneousTransforms::Type_fr_Link2_X_fr_Link1::Type_fr_Link2_X_fr_Link1()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link2_X_fr_Link1& HomogeneousTransforms::Type_fr_Link2_X_fr_Link1::update(const state_t& q)
{
    Scalar sin_q_joint2  = ScalarTraits::sin( q(JOINT2) );
    Scalar cos_q_joint2  = ScalarTraits::cos( q(JOINT2) );
    (*this)(0,0) = cos_q_joint2;
    (*this)(0,2) = sin_q_joint2;
    (*this)(1,0) = -sin_q_joint2;
    (*this)(1,2) = cos_q_joint2;
    return *this;
}
HomogeneousTransforms::Type_fr_Link1_X_fr_Link2::Type_fr_Link1_X_fr_Link2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = -1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link1_X_fr_Link2& HomogeneousTransforms::Type_fr_Link1_X_fr_Link2::update(const state_t& q)
{
    Scalar sin_q_joint2  = ScalarTraits::sin( q(JOINT2) );
    Scalar cos_q_joint2  = ScalarTraits::cos( q(JOINT2) );
    (*this)(0,0) = cos_q_joint2;
    (*this)(0,1) = -sin_q_joint2;
    (*this)(2,0) = sin_q_joint2;
    (*this)(2,1) = cos_q_joint2;
    return *this;
}
HomogeneousTransforms::Type_fr_Link3_X_fr_Link2::Type_fr_Link3_X_fr_Link2()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link3_X_fr_Link2& HomogeneousTransforms::Type_fr_Link3_X_fr_Link2::update(const state_t& q)
{
    Scalar sin_q_joint3  = ScalarTraits::sin( q(JOINT3) );
    Scalar cos_q_joint3  = ScalarTraits::cos( q(JOINT3) );
    (*this)(0,0) = cos_q_joint3;
    (*this)(0,1) = sin_q_joint3;
    (*this)(0,3) = (- ty_joint3 * sin_q_joint3)-( tx_joint3 * cos_q_joint3);
    (*this)(1,0) = -sin_q_joint3;
    (*this)(1,1) = cos_q_joint3;
    (*this)(1,3) = ( tx_joint3 * sin_q_joint3)-( ty_joint3 * cos_q_joint3);
    return *this;
}
HomogeneousTransforms::Type_fr_Link2_X_fr_Link3::Type_fr_Link2_X_fr_Link3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_joint3;    // Maxima DSL: _k__tx_joint3
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_joint3;    // Maxima DSL: _k__ty_joint3
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link2_X_fr_Link3& HomogeneousTransforms::Type_fr_Link2_X_fr_Link3::update(const state_t& q)
{
    Scalar sin_q_joint3  = ScalarTraits::sin( q(JOINT3) );
    Scalar cos_q_joint3  = ScalarTraits::cos( q(JOINT3) );
    (*this)(0,0) = cos_q_joint3;
    (*this)(0,1) = -sin_q_joint3;
    (*this)(1,0) = sin_q_joint3;
    (*this)(1,1) = cos_q_joint3;
    return *this;
}
HomogeneousTransforms::Type_fr_Link4_X_fr_Link3::Type_fr_Link4_X_fr_Link3()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(2,2) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
    (*this)(2,3) =  ty_joint4 * sin_rx_joint4;    // Maxima DSL: _k__ty_joint4*sin(_k__rx_joint4)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link4_X_fr_Link3& HomogeneousTransforms::Type_fr_Link4_X_fr_Link3::update(const state_t& q)
{
    Scalar sin_q_joint4  = ScalarTraits::sin( q(JOINT4) );
    Scalar cos_q_joint4  = ScalarTraits::cos( q(JOINT4) );
    (*this)(0,0) = cos_q_joint4;
    (*this)(0,1) = cos_rx_joint4 * sin_q_joint4;
    (*this)(0,2) = sin_rx_joint4 * sin_q_joint4;
    (*this)(0,3) = (- ty_joint4 * cos_rx_joint4 * sin_q_joint4)-( tx_joint4 * cos_q_joint4);
    (*this)(1,0) = -sin_q_joint4;
    (*this)(1,1) = cos_rx_joint4 * cos_q_joint4;
    (*this)(1,2) = sin_rx_joint4 * cos_q_joint4;
    (*this)(1,3) = ( tx_joint4 * sin_q_joint4)-( ty_joint4 * cos_rx_joint4 * cos_q_joint4);
    return *this;
}
HomogeneousTransforms::Type_fr_Link3_X_fr_Link4::Type_fr_Link3_X_fr_Link4()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_joint4;    // Maxima DSL: _k__tx_joint4
    (*this)(1,2) = -sin_rx_joint4;    // Maxima DSL: -sin(_k__rx_joint4)
    (*this)(1,3) =  ty_joint4;    // Maxima DSL: _k__ty_joint4
    (*this)(2,2) = cos_rx_joint4;    // Maxima DSL: cos(_k__rx_joint4)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link3_X_fr_Link4& HomogeneousTransforms::Type_fr_Link3_X_fr_Link4::update(const state_t& q)
{
    Scalar sin_q_joint4  = ScalarTraits::sin( q(JOINT4) );
    Scalar cos_q_joint4  = ScalarTraits::cos( q(JOINT4) );
    (*this)(0,0) = cos_q_joint4;
    (*this)(0,1) = -sin_q_joint4;
    (*this)(1,0) = cos_rx_joint4 * sin_q_joint4;
    (*this)(1,1) = cos_rx_joint4 * cos_q_joint4;
    (*this)(2,0) = sin_rx_joint4 * sin_q_joint4;
    (*this)(2,1) = sin_rx_joint4 * cos_q_joint4;
    return *this;
}
HomogeneousTransforms::Type_fr_Link5_X_fr_Link4::Type_fr_Link5_X_fr_Link4()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(2,2) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link5_X_fr_Link4& HomogeneousTransforms::Type_fr_Link5_X_fr_Link4::update(const state_t& q)
{
    Scalar sin_q_joint5  = ScalarTraits::sin( q(JOINT5) );
    Scalar cos_q_joint5  = ScalarTraits::cos( q(JOINT5) );
    (*this)(0,0) = cos_q_joint5;
    (*this)(0,1) = cos_rx_joint5 * sin_q_joint5;
    (*this)(0,2) = sin_rx_joint5 * sin_q_joint5;
    (*this)(1,0) = -sin_q_joint5;
    (*this)(1,1) = cos_rx_joint5 * cos_q_joint5;
    (*this)(1,2) = sin_rx_joint5 * cos_q_joint5;
    return *this;
}
HomogeneousTransforms::Type_fr_Link4_X_fr_Link5::Type_fr_Link4_X_fr_Link5()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = -sin_rx_joint5;    // Maxima DSL: -sin(_k__rx_joint5)
    (*this)(1,3) = 0.0;
    (*this)(2,2) = cos_rx_joint5;    // Maxima DSL: cos(_k__rx_joint5)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link4_X_fr_Link5& HomogeneousTransforms::Type_fr_Link4_X_fr_Link5::update(const state_t& q)
{
    Scalar sin_q_joint5  = ScalarTraits::sin( q(JOINT5) );
    Scalar cos_q_joint5  = ScalarTraits::cos( q(JOINT5) );
    (*this)(0,0) = cos_q_joint5;
    (*this)(0,1) = -sin_q_joint5;
    (*this)(1,0) = cos_rx_joint5 * sin_q_joint5;
    (*this)(1,1) = cos_rx_joint5 * cos_q_joint5;
    (*this)(2,0) = sin_rx_joint5 * sin_q_joint5;
    (*this)(2,1) = sin_rx_joint5 * cos_q_joint5;
    return *this;
}
HomogeneousTransforms::Type_fr_Link6_X_fr_Link5::Type_fr_Link6_X_fr_Link5()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(2,2) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
    (*this)(2,3) =  ty_joint6 * sin_rx_joint6;    // Maxima DSL: _k__ty_joint6*sin(_k__rx_joint6)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link6_X_fr_Link5& HomogeneousTransforms::Type_fr_Link6_X_fr_Link5::update(const state_t& q)
{
    Scalar sin_q_joint6  = ScalarTraits::sin( q(JOINT6) );
    Scalar cos_q_joint6  = ScalarTraits::cos( q(JOINT6) );
    (*this)(0,0) = cos_q_joint6;
    (*this)(0,1) = cos_rx_joint6 * sin_q_joint6;
    (*this)(0,2) = sin_rx_joint6 * sin_q_joint6;
    (*this)(0,3) = (- ty_joint6 * cos_rx_joint6 * sin_q_joint6)-( tx_joint6 * cos_q_joint6);
    (*this)(1,0) = -sin_q_joint6;
    (*this)(1,1) = cos_rx_joint6 * cos_q_joint6;
    (*this)(1,2) = sin_rx_joint6 * cos_q_joint6;
    (*this)(1,3) = ( tx_joint6 * sin_q_joint6)-( ty_joint6 * cos_rx_joint6 * cos_q_joint6);
    return *this;
}
HomogeneousTransforms::Type_fr_Link5_X_fr_Link6::Type_fr_Link5_X_fr_Link6()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_joint6;    // Maxima DSL: _k__tx_joint6
    (*this)(1,2) = -sin_rx_joint6;    // Maxima DSL: -sin(_k__rx_joint6)
    (*this)(1,3) =  ty_joint6;    // Maxima DSL: _k__ty_joint6
    (*this)(2,2) = cos_rx_joint6;    // Maxima DSL: cos(_k__rx_joint6)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_Link5_X_fr_Link6& HomogeneousTransforms::Type_fr_Link5_X_fr_Link6::update(const state_t& q)
{
    Scalar sin_q_joint6  = ScalarTraits::sin( q(JOINT6) );
    Scalar cos_q_joint6  = ScalarTraits::cos( q(JOINT6) );
    (*this)(0,0) = cos_q_joint6;
    (*this)(0,1) = -sin_q_joint6;
    (*this)(1,0) = cos_rx_joint6 * sin_q_joint6;
    (*this)(1,1) = cos_rx_joint6 * cos_q_joint6;
    (*this)(2,0) = sin_rx_joint6 * sin_q_joint6;
    (*this)(2,1) = sin_rx_joint6 * cos_q_joint6;
    return *this;
}

