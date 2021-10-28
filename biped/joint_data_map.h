#ifndef RCG_BIPED_JOINT_DATA_MAP_H_
#define RCG_BIPED_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace Biped {
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
            data[L_HAA] = rhs[L_HAA];
            data[L_HFE] = rhs[L_HFE];
            data[L_KFE] = rhs[L_KFE];
            data[R_HAA] = rhs[R_HAA];
            data[R_HFE] = rhs[R_HFE];
            data[R_KFE] = rhs[R_KFE];
        }

        template<typename T>
        inline
        void JointDataMap<T>::assigndata(const T& value) {
            data[L_HAA] = value;
            data[L_HFE] = value;
            data[L_KFE] = value;
            data[R_HAA] = value;
            data[R_HFE] = value;
            data[R_KFE] = value;
        }

        template<typename T>
        inline
        std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
            out
                    << "   L_HAA = "
                    << map[L_HAA]
                    << "   L_HFE = "
                    << map[L_HFE]
                    << "   L_KFE = "
                    << map[L_KFE]
                    << "   R_HAA = "
                    << map[R_HAA]
                    << "   R_HFE = "
                    << map[R_HFE]
                    << "   R_KFE = "
                    << map[R_KFE];
            return out;
        }

    }
}
#endif
