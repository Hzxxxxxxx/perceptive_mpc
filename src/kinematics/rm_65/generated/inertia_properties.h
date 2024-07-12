#ifndef RCG_RM_65_EXAMPLE_INERTIA_PROPERTIES_H_
#define RCG_RM_65_EXAMPLE_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace rm_65_example {
namespace rcg {

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_base_link() const;
        const InertiaMatrix& getTensor_Link1() const;
        const InertiaMatrix& getTensor_Link2() const;
        const InertiaMatrix& getTensor_Link3() const;
        const InertiaMatrix& getTensor_Link4() const;
        const InertiaMatrix& getTensor_Link5() const;
        const InertiaMatrix& getTensor_Link6() const;
        Scalar getMass_base_link() const;
        Scalar getMass_Link1() const;
        Scalar getMass_Link2() const;
        Scalar getMass_Link3() const;
        Scalar getMass_Link4() const;
        Scalar getMass_Link5() const;
        Scalar getMass_Link6() const;
        const Vector3& getCOM_base_link() const;
        const Vector3& getCOM_Link1() const;
        const Vector3& getCOM_Link2() const;
        const Vector3& getCOM_Link3() const;
        const Vector3& getCOM_Link4() const;
        const Vector3& getCOM_Link5() const;
        const Vector3& getCOM_Link6() const;
        Scalar getTotalMass() const;


        /*!
         * Fresh values for the runtime parameters of the robot rm_65_example,
         * causing the update of the inertia properties modeled by this
         * instance.
         */
        void updateParameters(const RuntimeInertiaParams&);

    private:
        RuntimeInertiaParams params;

        InertiaMatrix tensor_base_link;
        InertiaMatrix tensor_Link1;
        InertiaMatrix tensor_Link2;
        InertiaMatrix tensor_Link3;
        InertiaMatrix tensor_Link4;
        InertiaMatrix tensor_Link5;
        InertiaMatrix tensor_Link6;
        Vector3 com_base_link;
        Vector3 com_Link1;
        Vector3 com_Link2;
        Vector3 com_Link3;
        Vector3 com_Link4;
        Vector3 com_Link5;
        Vector3 com_Link6;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_base_link() const {
    return this->tensor_base_link;
}
inline const InertiaMatrix& InertiaProperties::getTensor_Link1() const {
    return this->tensor_Link1;
}
inline const InertiaMatrix& InertiaProperties::getTensor_Link2() const {
    return this->tensor_Link2;
}
inline const InertiaMatrix& InertiaProperties::getTensor_Link3() const {
    return this->tensor_Link3;
}
inline const InertiaMatrix& InertiaProperties::getTensor_Link4() const {
    return this->tensor_Link4;
}
inline const InertiaMatrix& InertiaProperties::getTensor_Link5() const {
    return this->tensor_Link5;
}
inline const InertiaMatrix& InertiaProperties::getTensor_Link6() const {
    return this->tensor_Link6;
}
inline Scalar InertiaProperties::getMass_base_link() const {
    return this->tensor_base_link.getMass();
}
inline Scalar InertiaProperties::getMass_Link1() const {
    return this->tensor_Link1.getMass();
}
inline Scalar InertiaProperties::getMass_Link2() const {
    return this->tensor_Link2.getMass();
}
inline Scalar InertiaProperties::getMass_Link3() const {
    return this->tensor_Link3.getMass();
}
inline Scalar InertiaProperties::getMass_Link4() const {
    return this->tensor_Link4.getMass();
}
inline Scalar InertiaProperties::getMass_Link5() const {
    return this->tensor_Link5.getMass();
}
inline Scalar InertiaProperties::getMass_Link6() const {
    return this->tensor_Link6.getMass();
}
inline const Vector3& InertiaProperties::getCOM_base_link() const {
    return this->com_base_link;
}
inline const Vector3& InertiaProperties::getCOM_Link1() const {
    return this->com_Link1;
}
inline const Vector3& InertiaProperties::getCOM_Link2() const {
    return this->com_Link2;
}
inline const Vector3& InertiaProperties::getCOM_Link3() const {
    return this->com_Link3;
}
inline const Vector3& InertiaProperties::getCOM_Link4() const {
    return this->com_Link4;
}
inline const Vector3& InertiaProperties::getCOM_Link5() const {
    return this->com_Link5;
}
inline const Vector3& InertiaProperties::getCOM_Link6() const {
    return this->com_Link6;
}

inline Scalar InertiaProperties::getTotalMass() const {
    return m_base_link + m_Link1 + m_Link2 + m_Link3 + m_Link4 + m_Link5 + m_Link6;
}

}
}

#endif
