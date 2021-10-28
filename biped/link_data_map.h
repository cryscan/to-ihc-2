#ifndef RCG_BIPED_LINK_DATA_MAP_H_
#define RCG_BIPED_LINK_DATA_MAP_H_

#include "declarations.h"

namespace Biped {
    namespace rcg {

/**
 * A very simple container to associate a generic data item to each link
 */
        template<typename T>
        class LinkDataMap {
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

        template<typename T>
        inline
        LinkDataMap<T>::LinkDataMap(const T& value) {
            assigndata(value);
        }

        template<typename T>
        inline
        LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs) {
            copydata(rhs);
        }

        template<typename T>
        inline
        LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs) {
            if (&rhs != this) {
                copydata(rhs);
            }
            return *this;
        }

        template<typename T>
        inline
        LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value) {
            assigndata(value);
            return *this;
        }

        template<typename T>
        inline
        T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
            return data[l];
        }

        template<typename T>
        inline
        const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
            return data[l];
        }

        template<typename T>
        inline
        void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
            data[TRUNK] = rhs[TRUNK];
            data[L_HIP] = rhs[L_HIP];
            data[L_THIGH] = rhs[L_THIGH];
            data[L_SHIN] = rhs[L_SHIN];
            data[R_HIP] = rhs[R_HIP];
            data[R_THIGH] = rhs[R_THIGH];
            data[R_SHIN] = rhs[R_SHIN];
        }

        template<typename T>
        inline
        void LinkDataMap<T>::assigndata(const T& value) {
            data[TRUNK] = value;
            data[L_HIP] = value;
            data[L_THIGH] = value;
            data[L_SHIN] = value;
            data[R_HIP] = value;
            data[R_THIGH] = value;
            data[R_SHIN] = value;
        }

        template<typename T>
        inline
        std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
            out
                    << "   trunk = "
                    << map[TRUNK]
                    << "   L_hip = "
                    << map[L_HIP]
                    << "   L_thigh = "
                    << map[L_THIGH]
                    << "   L_shin = "
                    << map[L_SHIN]
                    << "   R_hip = "
                    << map[R_HIP]
                    << "   R_thigh = "
                    << map[R_THIGH]
                    << "   R_shin = "
                    << map[R_SHIN];
            return out;
        }

    }
}
#endif
