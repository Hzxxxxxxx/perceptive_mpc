#ifndef RCG_RM_65_EXAMPLE_LINK_DATA_MAP_H_
#define RCG_RM_65_EXAMPLE_LINK_DATA_MAP_H_

#include "declarations.h"

namespace rm_65_example {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[BASE_LINK] = rhs[BASE_LINK];
    data[LINK1] = rhs[LINK1];
    data[LINK2] = rhs[LINK2];
    data[LINK3] = rhs[LINK3];
    data[LINK4] = rhs[LINK4];
    data[LINK5] = rhs[LINK5];
    data[LINK6] = rhs[LINK6];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[BASE_LINK] = value;
    data[LINK1] = value;
    data[LINK2] = value;
    data[LINK3] = value;
    data[LINK4] = value;
    data[LINK5] = value;
    data[LINK6] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   base_link = "
    << map[BASE_LINK]
    << "   Link1 = "
    << map[LINK1]
    << "   Link2 = "
    << map[LINK2]
    << "   Link3 = "
    << map[LINK3]
    << "   Link4 = "
    << map[LINK4]
    << "   Link5 = "
    << map[LINK5]
    << "   Link6 = "
    << map[LINK6]
    ;
    return out;
}

}
}
#endif
