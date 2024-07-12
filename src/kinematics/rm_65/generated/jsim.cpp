#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
rm_65_example::rcg::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    Link6_Ic(linkInertias.getTensor_Link6())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
#define Fcol(j) (block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)
const rm_65_example::rcg::JSIM& rm_65_example::rcg::JSIM::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_Link5_X_fr_Link6(state);
    frcTransf -> fr_Link4_X_fr_Link5(state);
    frcTransf -> fr_Link3_X_fr_Link4(state);
    frcTransf -> fr_Link2_X_fr_Link3(state);
    frcTransf -> fr_Link1_X_fr_Link2(state);
    frcTransf -> fr_base_link_X_fr_Link1(state);

    // Initializes the composite inertia tensors
    base_link_Ic = linkInertias.getTensor_base_link();
    Link1_Ic = linkInertias.getTensor_Link1();
    Link2_Ic = linkInertias.getTensor_Link2();
    Link3_Ic = linkInertias.getTensor_Link3();
    Link4_Ic = linkInertias.getTensor_Link4();
    Link5_Ic = linkInertias.getTensor_Link5();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link Link6:
    iit::rbd::transformInertia<Scalar>(Link6_Ic, frcTransf -> fr_Link5_X_fr_Link6, Ic_spare);
    Link5_Ic += Ic_spare;

    Fcol(JOINT6) = Link6_Ic.col(AZ);
    DATA(JOINT6+6, JOINT6+6) = Fcol(JOINT6)(AZ);

    Fcol(JOINT6) = frcTransf -> fr_Link5_X_fr_Link6 * Fcol(JOINT6);
    DATA(JOINT6+6, JOINT5+6) = F(AZ,JOINT6);
    DATA(JOINT5+6, JOINT6+6) = DATA(JOINT6+6, JOINT5+6);
    Fcol(JOINT6) = frcTransf -> fr_Link4_X_fr_Link5 * Fcol(JOINT6);
    DATA(JOINT6+6, JOINT4+6) = F(AZ,JOINT6);
    DATA(JOINT4+6, JOINT6+6) = DATA(JOINT6+6, JOINT4+6);
    Fcol(JOINT6) = frcTransf -> fr_Link3_X_fr_Link4 * Fcol(JOINT6);
    DATA(JOINT6+6, JOINT3+6) = F(AZ,JOINT6);
    DATA(JOINT3+6, JOINT6+6) = DATA(JOINT6+6, JOINT3+6);
    Fcol(JOINT6) = frcTransf -> fr_Link2_X_fr_Link3 * Fcol(JOINT6);
    DATA(JOINT6+6, JOINT2+6) = F(AZ,JOINT6);
    DATA(JOINT2+6, JOINT6+6) = DATA(JOINT6+6, JOINT2+6);
    Fcol(JOINT6) = frcTransf -> fr_Link1_X_fr_Link2 * Fcol(JOINT6);
    DATA(JOINT6+6, JOINT1+6) = F(AZ,JOINT6);
    DATA(JOINT1+6, JOINT6+6) = DATA(JOINT6+6, JOINT1+6);
    Fcol(JOINT6) = frcTransf -> fr_base_link_X_fr_Link1 * Fcol(JOINT6);

    // Link Link5:
    iit::rbd::transformInertia<Scalar>(Link5_Ic, frcTransf -> fr_Link4_X_fr_Link5, Ic_spare);
    Link4_Ic += Ic_spare;

    Fcol(JOINT5) = Link5_Ic.col(AZ);
    DATA(JOINT5+6, JOINT5+6) = Fcol(JOINT5)(AZ);

    Fcol(JOINT5) = frcTransf -> fr_Link4_X_fr_Link5 * Fcol(JOINT5);
    DATA(JOINT5+6, JOINT4+6) = F(AZ,JOINT5);
    DATA(JOINT4+6, JOINT5+6) = DATA(JOINT5+6, JOINT4+6);
    Fcol(JOINT5) = frcTransf -> fr_Link3_X_fr_Link4 * Fcol(JOINT5);
    DATA(JOINT5+6, JOINT3+6) = F(AZ,JOINT5);
    DATA(JOINT3+6, JOINT5+6) = DATA(JOINT5+6, JOINT3+6);
    Fcol(JOINT5) = frcTransf -> fr_Link2_X_fr_Link3 * Fcol(JOINT5);
    DATA(JOINT5+6, JOINT2+6) = F(AZ,JOINT5);
    DATA(JOINT2+6, JOINT5+6) = DATA(JOINT5+6, JOINT2+6);
    Fcol(JOINT5) = frcTransf -> fr_Link1_X_fr_Link2 * Fcol(JOINT5);
    DATA(JOINT5+6, JOINT1+6) = F(AZ,JOINT5);
    DATA(JOINT1+6, JOINT5+6) = DATA(JOINT5+6, JOINT1+6);
    Fcol(JOINT5) = frcTransf -> fr_base_link_X_fr_Link1 * Fcol(JOINT5);

    // Link Link4:
    iit::rbd::transformInertia<Scalar>(Link4_Ic, frcTransf -> fr_Link3_X_fr_Link4, Ic_spare);
    Link3_Ic += Ic_spare;

    Fcol(JOINT4) = Link4_Ic.col(AZ);
    DATA(JOINT4+6, JOINT4+6) = Fcol(JOINT4)(AZ);

    Fcol(JOINT4) = frcTransf -> fr_Link3_X_fr_Link4 * Fcol(JOINT4);
    DATA(JOINT4+6, JOINT3+6) = F(AZ,JOINT4);
    DATA(JOINT3+6, JOINT4+6) = DATA(JOINT4+6, JOINT3+6);
    Fcol(JOINT4) = frcTransf -> fr_Link2_X_fr_Link3 * Fcol(JOINT4);
    DATA(JOINT4+6, JOINT2+6) = F(AZ,JOINT4);
    DATA(JOINT2+6, JOINT4+6) = DATA(JOINT4+6, JOINT2+6);
    Fcol(JOINT4) = frcTransf -> fr_Link1_X_fr_Link2 * Fcol(JOINT4);
    DATA(JOINT4+6, JOINT1+6) = F(AZ,JOINT4);
    DATA(JOINT1+6, JOINT4+6) = DATA(JOINT4+6, JOINT1+6);
    Fcol(JOINT4) = frcTransf -> fr_base_link_X_fr_Link1 * Fcol(JOINT4);

    // Link Link3:
    iit::rbd::transformInertia<Scalar>(Link3_Ic, frcTransf -> fr_Link2_X_fr_Link3, Ic_spare);
    Link2_Ic += Ic_spare;

    Fcol(JOINT3) = Link3_Ic.col(AZ);
    DATA(JOINT3+6, JOINT3+6) = Fcol(JOINT3)(AZ);

    Fcol(JOINT3) = frcTransf -> fr_Link2_X_fr_Link3 * Fcol(JOINT3);
    DATA(JOINT3+6, JOINT2+6) = F(AZ,JOINT3);
    DATA(JOINT2+6, JOINT3+6) = DATA(JOINT3+6, JOINT2+6);
    Fcol(JOINT3) = frcTransf -> fr_Link1_X_fr_Link2 * Fcol(JOINT3);
    DATA(JOINT3+6, JOINT1+6) = F(AZ,JOINT3);
    DATA(JOINT1+6, JOINT3+6) = DATA(JOINT3+6, JOINT1+6);
    Fcol(JOINT3) = frcTransf -> fr_base_link_X_fr_Link1 * Fcol(JOINT3);

    // Link Link2:
    iit::rbd::transformInertia<Scalar>(Link2_Ic, frcTransf -> fr_Link1_X_fr_Link2, Ic_spare);
    Link1_Ic += Ic_spare;

    Fcol(JOINT2) = Link2_Ic.col(AZ);
    DATA(JOINT2+6, JOINT2+6) = Fcol(JOINT2)(AZ);

    Fcol(JOINT2) = frcTransf -> fr_Link1_X_fr_Link2 * Fcol(JOINT2);
    DATA(JOINT2+6, JOINT1+6) = F(AZ,JOINT2);
    DATA(JOINT1+6, JOINT2+6) = DATA(JOINT2+6, JOINT1+6);
    Fcol(JOINT2) = frcTransf -> fr_base_link_X_fr_Link1 * Fcol(JOINT2);

    // Link Link1:
    iit::rbd::transformInertia<Scalar>(Link1_Ic, frcTransf -> fr_base_link_X_fr_Link1, Ic_spare);
    base_link_Ic += Ic_spare;

    Fcol(JOINT1) = Link1_Ic.col(AZ);
    DATA(JOINT1+6, JOINT1+6) = Fcol(JOINT1)(AZ);

    Fcol(JOINT1) = frcTransf -> fr_base_link_X_fr_Link1 * Fcol(JOINT1);

    // Copies the upper-right block into the lower-left block, after transposing
    block<6, 6>(6,0) = (block<6, 6>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    block<6,6>(0,0) = base_link_Ic;
    return *this;
}

#undef DATA
#undef F

void rm_65_example::rcg::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint joint6, index 5 :
    L(5, 5) = ScalarTraits::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(5, 2) = L(5, 2) / L(5, 5);
    L(5, 1) = L(5, 1) / L(5, 5);
    L(5, 0) = L(5, 0) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(4, 2) = L(4, 2) - L(5, 4) * L(5, 2);
    L(4, 1) = L(4, 1) - L(5, 4) * L(5, 1);
    L(4, 0) = L(4, 0) - L(5, 4) * L(5, 0);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    L(3, 2) = L(3, 2) - L(5, 3) * L(5, 2);
    L(3, 1) = L(3, 1) - L(5, 3) * L(5, 1);
    L(3, 0) = L(3, 0) - L(5, 3) * L(5, 0);
    L(2, 2) = L(2, 2) - L(5, 2) * L(5, 2);
    L(2, 1) = L(2, 1) - L(5, 2) * L(5, 1);
    L(2, 0) = L(2, 0) - L(5, 2) * L(5, 0);
    L(1, 1) = L(1, 1) - L(5, 1) * L(5, 1);
    L(1, 0) = L(1, 0) - L(5, 1) * L(5, 0);
    L(0, 0) = L(0, 0) - L(5, 0) * L(5, 0);
    
    // Joint joint5, index 4 :
    L(4, 4) = ScalarTraits::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(4, 2) = L(4, 2) / L(4, 4);
    L(4, 1) = L(4, 1) / L(4, 4);
    L(4, 0) = L(4, 0) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    L(3, 2) = L(3, 2) - L(4, 3) * L(4, 2);
    L(3, 1) = L(3, 1) - L(4, 3) * L(4, 1);
    L(3, 0) = L(3, 0) - L(4, 3) * L(4, 0);
    L(2, 2) = L(2, 2) - L(4, 2) * L(4, 2);
    L(2, 1) = L(2, 1) - L(4, 2) * L(4, 1);
    L(2, 0) = L(2, 0) - L(4, 2) * L(4, 0);
    L(1, 1) = L(1, 1) - L(4, 1) * L(4, 1);
    L(1, 0) = L(1, 0) - L(4, 1) * L(4, 0);
    L(0, 0) = L(0, 0) - L(4, 0) * L(4, 0);
    
    // Joint joint4, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));
    L(3, 2) = L(3, 2) / L(3, 3);
    L(3, 1) = L(3, 1) / L(3, 3);
    L(3, 0) = L(3, 0) / L(3, 3);
    L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
    L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
    L(2, 0) = L(2, 0) - L(3, 2) * L(3, 0);
    L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);
    L(1, 0) = L(1, 0) - L(3, 1) * L(3, 0);
    L(0, 0) = L(0, 0) - L(3, 0) * L(3, 0);
    
    // Joint joint3, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint joint2, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint joint1, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));
    
}

void rm_65_example::rcg::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 0) * Linv(3, 0)) + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) + (Linv(3, 3) * Linv(3, 3));
    inverse(3, 2) =  + (Linv(3, 0) * Linv(2, 0)) + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =  + (Linv(3, 0) * Linv(1, 0)) + (Linv(3, 1) * Linv(1, 1));
    inverse(1, 3) = inverse(3, 1);
    inverse(3, 0) =  + (Linv(3, 0) * Linv(0, 0));
    inverse(0, 3) = inverse(3, 0);
    inverse(4, 4) =  + (Linv(4, 0) * Linv(4, 0)) + (Linv(4, 1) * Linv(4, 1)) + (Linv(4, 2) * Linv(4, 2)) + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 0) * Linv(3, 0)) + (Linv(4, 1) * Linv(3, 1)) + (Linv(4, 2) * Linv(3, 2)) + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 2) =  + (Linv(4, 0) * Linv(2, 0)) + (Linv(4, 1) * Linv(2, 1)) + (Linv(4, 2) * Linv(2, 2));
    inverse(2, 4) = inverse(4, 2);
    inverse(4, 1) =  + (Linv(4, 0) * Linv(1, 0)) + (Linv(4, 1) * Linv(1, 1));
    inverse(1, 4) = inverse(4, 1);
    inverse(4, 0) =  + (Linv(4, 0) * Linv(0, 0));
    inverse(0, 4) = inverse(4, 0);
    inverse(5, 5) =  + (Linv(5, 0) * Linv(5, 0)) + (Linv(5, 1) * Linv(5, 1)) + (Linv(5, 2) * Linv(5, 2)) + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 0) * Linv(4, 0)) + (Linv(5, 1) * Linv(4, 1)) + (Linv(5, 2) * Linv(4, 2)) + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 0) * Linv(3, 0)) + (Linv(5, 1) * Linv(3, 1)) + (Linv(5, 2) * Linv(3, 2)) + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(5, 2) =  + (Linv(5, 0) * Linv(2, 0)) + (Linv(5, 1) * Linv(2, 1)) + (Linv(5, 2) * Linv(2, 2));
    inverse(2, 5) = inverse(5, 2);
    inverse(5, 1) =  + (Linv(5, 0) * Linv(1, 0)) + (Linv(5, 1) * Linv(1, 1));
    inverse(1, 5) = inverse(5, 1);
    inverse(5, 0) =  + (Linv(5, 0) * Linv(0, 0));
    inverse(0, 5) = inverse(5, 0);
}

void rm_65_example::rcg::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
    Linv(3, 0) = - Linv(0, 0) * ((Linv(3, 1) * L(1, 0)) + (Linv(3, 2) * L(2, 0)) + (Linv(3, 3) * L(3, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(4, 2) = - Linv(2, 2) * ((Linv(4, 3) * L(3, 2)) + (Linv(4, 4) * L(4, 2)) + 0);
    Linv(4, 1) = - Linv(1, 1) * ((Linv(4, 2) * L(2, 1)) + (Linv(4, 3) * L(3, 1)) + (Linv(4, 4) * L(4, 1)) + 0);
    Linv(4, 0) = - Linv(0, 0) * ((Linv(4, 1) * L(1, 0)) + (Linv(4, 2) * L(2, 0)) + (Linv(4, 3) * L(3, 0)) + (Linv(4, 4) * L(4, 0)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(5, 2) = - Linv(2, 2) * ((Linv(5, 3) * L(3, 2)) + (Linv(5, 4) * L(4, 2)) + (Linv(5, 5) * L(5, 2)) + 0);
    Linv(5, 1) = - Linv(1, 1) * ((Linv(5, 2) * L(2, 1)) + (Linv(5, 3) * L(3, 1)) + (Linv(5, 4) * L(4, 1)) + (Linv(5, 5) * L(5, 1)) + 0);
    Linv(5, 0) = - Linv(0, 0) * ((Linv(5, 1) * L(1, 0)) + (Linv(5, 2) * L(2, 0)) + (Linv(5, 3) * L(3, 0)) + (Linv(5, 4) * L(4, 0)) + (Linv(5, 5) * L(5, 0)) + 0);
}
