#ifndef RCG_HOPPER_INERTIA_PROPERTIES_H_
#define RCG_HOPPER_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace Hopper {
    namespace rcg {

        class InertiaProperties {
        public:
            InertiaProperties();
            ~InertiaProperties();
            const InertiaMatrix& getTensor_u1() const;
            const InertiaMatrix& getTensor_u2() const;
            const InertiaMatrix& getTensor_body() const;
            const InertiaMatrix& getTensor_leg() const;
            Scalar getMass_u1() const;
            Scalar getMass_u2() const;
            Scalar getMass_body() const;
            Scalar getMass_leg() const;
            const Vector3& getCOM_u1() const;
            const Vector3& getCOM_u2() const;
            const Vector3& getCOM_body() const;
            const Vector3& getCOM_leg() const;
            Scalar getTotalMass() const;


            /*!
             * Fresh values for the runtime parameters of the robot Hopper,
             * causing the update of the inertia properties modeled by this
             * instance.
             */
            void updateParameters(const RuntimeInertiaParams&);

        private:
            RuntimeInertiaParams params;

            InertiaMatrix tensor_u1;
            InertiaMatrix tensor_u2;
            InertiaMatrix tensor_body;
            InertiaMatrix tensor_leg;
            Vector3 com_u1;
            Vector3 com_u2;
            Vector3 com_body;
            Vector3 com_leg;
        };


        inline InertiaProperties::~InertiaProperties() {}

        inline const InertiaMatrix& InertiaProperties::getTensor_u1() const {
            return this->tensor_u1;
        }
        inline const InertiaMatrix& InertiaProperties::getTensor_u2() const {
            return this->tensor_u2;
        }
        inline const InertiaMatrix& InertiaProperties::getTensor_body() const {
            return this->tensor_body;
        }
        inline const InertiaMatrix& InertiaProperties::getTensor_leg() const {
            return this->tensor_leg;
        }
        inline Scalar InertiaProperties::getMass_u1() const {
            return this->tensor_u1.getMass();
        }
        inline Scalar InertiaProperties::getMass_u2() const {
            return this->tensor_u2.getMass();
        }
        inline Scalar InertiaProperties::getMass_body() const {
            return this->tensor_body.getMass();
        }
        inline Scalar InertiaProperties::getMass_leg() const {
            return this->tensor_leg.getMass();
        }
        inline const Vector3& InertiaProperties::getCOM_u1() const {
            return this->com_u1;
        }
        inline const Vector3& InertiaProperties::getCOM_u2() const {
            return this->com_u2;
        }
        inline const Vector3& InertiaProperties::getCOM_body() const {
            return this->com_body;
        }
        inline const Vector3& InertiaProperties::getCOM_leg() const {
            return this->com_leg;
        }

        inline Scalar InertiaProperties::getTotalMass() const {
            return 0.0 + 0.0 + m_body + m_leg;
        }

    }
}

#endif
