#ifndef RCG_RM_65_EXAMPLE_JOINT_DATA_MAP_H_
#define RCG_RM_65_EXAMPLE_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace rm_65_example {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[JOINT1] = rhs[JOINT1];
    data[JOINT2] = rhs[JOINT2];
    data[JOINT3] = rhs[JOINT3];
    data[JOINT4] = rhs[JOINT4];
    data[JOINT5] = rhs[JOINT5];
    data[JOINT6] = rhs[JOINT6];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[JOINT1] = value;
    data[JOINT2] = value;
    data[JOINT3] = value;
    data[JOINT4] = value;
    data[JOINT5] = value;
    data[JOINT6] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   joint1 = "
    << map[JOINT1]
    << "   joint2 = "
    << map[JOINT2]
    << "   joint3 = "
    << map[JOINT3]
    << "   joint4 = "
    << map[JOINT4]
    << "   joint5 = "
    << map[JOINT5]
    << "   joint6 = "
    << map[JOINT6]
    ;
    return out;
}

}
}
#endif
