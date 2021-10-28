#ifndef RCG_BIPED_INERTIA_PROPERTIES_H_
#define RCG_BIPED_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace Biped {
    namespace rcg {

        class InertiaProperties {
        public:
            InertiaProperties();
            ~InertiaProperties();
            const InertiaMatrix& getTensor_trunk() const;
            const InertiaMatrix& getTensor_L_hip() const;
            const InertiaMatrix& getTensor_L_thigh() const;
            const InertiaMatrix& getTensor_L_shin() const;
            const InertiaMatrix& getTensor_R_hip() const;
            const InertiaMatrix& getTensor_R_thigh() const;
            const InertiaMatrix& getTensor_R_shin() const;
            Scalar getMass_trunk() const;
            Scalar getMass_L_hip() const;
            Scalar getMass_L_thigh() const;
            Scalar getMass_L_shin() const;
            Scalar getMass_R_hip() const;
            Scalar getMass_R_thigh() const;
            Scalar getMass_R_shin() const;
            const Vector3& getCOM_trunk() const;
            const Vector3& getCOM_L_hip() const;
            const Vector3& getCOM_L_thigh() const;
            const Vector3& getCOM_L_shin() const;
            const Vector3& getCOM_R_hip() const;
            const Vector3& getCOM_R_thigh() const;
            const Vector3& getCOM_R_shin() const;
            Scalar getTotalMass() const;


            /*!
             * Fresh values for the runtime parameters of the robot Biped,
             * causing the update of the inertia properties modeled by this
             * instance.
             */
            void updateParameters(const RuntimeInertiaParams&);

        private:
            RuntimeInertiaParams params;

            InertiaMatrix tensor_trunk;
            InertiaMatrix tensor_L_hip;
            InertiaMatrix tensor_L_thigh;
            InertiaMatrix tensor_L_shin;
            InertiaMatrix tensor_R_hip;
            InertiaMatrix tensor_R_thigh;
            InertiaMatrix tensor_R_shin;
            Vector3 com_trunk;
            Vector3 com_L_hip;
            Vector3 com_L_thigh;
            Vector3 com_L_shin;
            Vector3 com_R_hip;
            Vector3 com_R_thigh;
            Vector3 com_R_shin;
        };


        inline InertiaProperties::~InertiaProperties() {}

        inline const InertiaMatrix& InertiaProperties::getTensor_trunk() const {
            return this->tensor_trunk;
        }
        inline const InertiaMatrix& InertiaProperties::getTensor_L_hip() const {
            return this->tensor_L_hip;
        }
        inline const InertiaMatrix& InertiaProperties::getTensor_L_thigh() const {
            return this->tensor_L_thigh;
        }
        inline const InertiaMatrix& InertiaProperties::getTensor_L_shin() const {
            return this->tensor_L_shin;
        }
        inline const InertiaMatrix& InertiaProperties::getTensor_R_hip() const {
            return this->tensor_R_hip;
        }
        inline const InertiaMatrix& InertiaProperties::getTensor_R_thigh() const {
            return this->tensor_R_thigh;
        }
        inline const InertiaMatrix& InertiaProperties::getTensor_R_shin() const {
            return this->tensor_R_shin;
        }
        inline Scalar InertiaProperties::getMass_trunk() const {
            return this->tensor_trunk.getMass();
        }
        inline Scalar InertiaProperties::getMass_L_hip() const {
            return this->tensor_L_hip.getMass();
        }
        inline Scalar InertiaProperties::getMass_L_thigh() const {
            return this->tensor_L_thigh.getMass();
        }
        inline Scalar InertiaProperties::getMass_L_shin() const {
            return this->tensor_L_shin.getMass();
        }
        inline Scalar InertiaProperties::getMass_R_hip() const {
            return this->tensor_R_hip.getMass();
        }
        inline Scalar InertiaProperties::getMass_R_thigh() const {
            return this->tensor_R_thigh.getMass();
        }
        inline Scalar InertiaProperties::getMass_R_shin() const {
            return this->tensor_R_shin.getMass();
        }
        inline const Vector3& InertiaProperties::getCOM_trunk() const {
            return this->com_trunk;
        }
        inline const Vector3& InertiaProperties::getCOM_L_hip() const {
            return this->com_L_hip;
        }
        inline const Vector3& InertiaProperties::getCOM_L_thigh() const {
            return this->com_L_thigh;
        }
        inline const Vector3& InertiaProperties::getCOM_L_shin() const {
            return this->com_L_shin;
        }
        inline const Vector3& InertiaProperties::getCOM_R_hip() const {
            return this->com_R_hip;
        }
        inline const Vector3& InertiaProperties::getCOM_R_thigh() const {
            return this->com_R_thigh;
        }
        inline const Vector3& InertiaProperties::getCOM_R_shin() const {
            return this->com_R_shin;
        }

        inline Scalar InertiaProperties::getTotalMass() const {
            return m_trunk + m_L_hip + m_L_thigh + m_L_shin + m_R_hip + m_R_thigh + m_R_shin;
        }

    }
}

#endif
