#ifndef RCG_HOPPER_JOINT_DATA_MAP_H_
#define RCG_HOPPER_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace Hopper {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each joint
 */
    template<typename T>
    class JointDataMap {
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

    template<typename T>
    inline
    JointDataMap<T>::JointDataMap(const T& value) {
        assigndata(value);
    }

    template<typename T>
    inline
    JointDataMap<T>::JointDataMap(const JointDataMap& rhs) {
        copydata(rhs);
    }

    template<typename T>
    inline
    JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs) {
        if (&rhs != this) {
            copydata(rhs);
        }
        return *this;
    }

    template<typename T>
    inline
    JointDataMap<T>& JointDataMap<T>::operator=(const T& value) {
        assigndata(value);
        return *this;
    }

    template<typename T>
    inline
    T& JointDataMap<T>::operator[](JointIdentifiers j) {
        return data[j];
    }

    template<typename T>
    inline
    const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
        return data[j];
    }

    template<typename T>
    inline
    void JointDataMap<T>::copydata(const JointDataMap& rhs) {
        data[BH] = rhs[BH];
        data[BX] = rhs[BX];
        data[HFE] = rhs[HFE];
        data[KFE] = rhs[KFE];
    }

    template<typename T>
    inline
    void JointDataMap<T>::assigndata(const T& value) {
        data[BH] = value;
        data[BX] = value;
        data[HFE] = value;
        data[KFE] = value;
    }

    template<typename T>
    inline
    std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
        out
                << "   BH = "
                << map[BH]
                << "   BX = "
                << map[BX]
                << "   HFE = "
                << map[HFE]
                << "   KFE = "
                << map[KFE];
        return out;
    }

}
}
#endif
