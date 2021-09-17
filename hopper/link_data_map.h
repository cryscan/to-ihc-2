#ifndef RCG_HOPPER_LINK_DATA_MAP_H_
#define RCG_HOPPER_LINK_DATA_MAP_H_

#include "declarations.h"

namespace Hopper {
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
            data[U0] = rhs[U0];
            data[U1] = rhs[U1];
            data[U2] = rhs[U2];
            data[BODY] = rhs[BODY];
            data[LEG] = rhs[LEG];
        }

        template<typename T>
        inline
        void LinkDataMap<T>::assigndata(const T& value) {
            data[U0] = value;
            data[U1] = value;
            data[U2] = value;
            data[BODY] = value;
            data[LEG] = value;
        }

        template<typename T>
        inline
        std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
            out
                    << "   u0 = "
                    << map[U0]
                    << "   u1 = "
                    << map[U1]
                    << "   u2 = "
                    << map[U2]
                    << "   body = "
                    << map[BODY]
                    << "   leg = "
                    << map[LEG];
            return out;
        }

    }
}
#endif
