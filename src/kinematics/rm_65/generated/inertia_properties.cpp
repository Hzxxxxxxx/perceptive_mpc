#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

rm_65_example::rcg::InertiaProperties::InertiaProperties()
{
    com_base_link = Vector3(comx_base_link,comy_base_link,comz_base_link);
    tensor_base_link.fill(
        m_base_link,
        com_base_link,
        Utils::buildInertiaTensor<Scalar>(ix_base_link,iy_base_link,iz_base_link,ixy_base_link,ixz_base_link,iyz_base_link) );

    com_Link1 = Vector3(comx_Link1,comy_Link1,comz_Link1);
    tensor_Link1.fill(
        m_Link1,
        com_Link1,
        Utils::buildInertiaTensor<Scalar>(ix_Link1,iy_Link1,iz_Link1,ixy_Link1,ixz_Link1,iyz_Link1) );

    com_Link2 = Vector3(comx_Link2,comy_Link2,comz_Link2);
    tensor_Link2.fill(
        m_Link2,
        com_Link2,
        Utils::buildInertiaTensor<Scalar>(ix_Link2,iy_Link2,iz_Link2,ixy_Link2,ixz_Link2,iyz_Link2) );

    com_Link3 = Vector3(comx_Link3,comy_Link3,comz_Link3);
    tensor_Link3.fill(
        m_Link3,
        com_Link3,
        Utils::buildInertiaTensor<Scalar>(ix_Link3,iy_Link3,iz_Link3,ixy_Link3,ixz_Link3,iyz_Link3) );

    com_Link4 = Vector3(comx_Link4,comy_Link4,comz_Link4);
    tensor_Link4.fill(
        m_Link4,
        com_Link4,
        Utils::buildInertiaTensor<Scalar>(ix_Link4,iy_Link4,iz_Link4,ixy_Link4,ixz_Link4,iyz_Link4) );

    com_Link5 = Vector3(comx_Link5,comy_Link5,comz_Link5);
    tensor_Link5.fill(
        m_Link5,
        com_Link5,
        Utils::buildInertiaTensor<Scalar>(ix_Link5,iy_Link5,iz_Link5,ixy_Link5,ixz_Link5,iyz_Link5) );

    com_Link6 = Vector3(comx_Link6,comy_Link6,comz_Link6);
    tensor_Link6.fill(
        m_Link6,
        com_Link6,
        Utils::buildInertiaTensor<Scalar>(ix_Link6,iy_Link6,iz_Link6,ixy_Link6,ixz_Link6,iyz_Link6) );

}


void rm_65_example::rcg::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}
