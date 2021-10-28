#ifndef RCG_BIPED_JSIM_H_
#define RCG_BIPED_JSIM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>

#include "rbd_types.h"
#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"

namespace Biped {
    namespace rcg {

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot Biped.
 */
        class JSIM : public iit::rbd::StateDependentMatrix<Biped::rcg::JointState, 12, 12, JSIM> {
        private:
            typedef iit::rbd::StateDependentMatrix<Biped::rcg::JointState, 12, 12, JSIM> Base;
        public:
            typedef Base::Scalar Scalar;
            typedef Base::Index Index;
            typedef Base::MatrixType MatrixType;
            /** The type of the F sub-block of the floating-base JSIM */
            typedef const iit::rbd::MatrixBlock<const MatrixType, 6, 6> BlockF_t;
            /** The type of the fixed-base sub-block of the JSIM */
            typedef const iit::rbd::MatrixBlock<const MatrixType, 6, 6> BlockFixedBase_t;
        public:
            JSIM(InertiaProperties&, ForceTransforms&);
            ~JSIM() {}

            const JSIM& update(const JointState&);


            /**
             * Computes and saves the matrix L of the L^T L factorization of this JSIM.
             */
            void computeL();
            /**
             * Computes and saves the inverse of this JSIM.
             * This function assumes that computeL() has been called already, since it
             * uses L to compute the inverse. The algorithm takes advantage of the branch
             * induced sparsity of the robot, if any.
             */
            void computeInverse();
            /**
             * Returns an unmodifiable reference to the matrix L. See also computeL()
             */
            const MatrixType& getL() const;
            /**
             * Returns an unmodifiable reference to the inverse of this JSIM
             */
            const MatrixType& getInverse() const;

            /**
             * The spatial composite-inertia tensor of the robot base,
             * ie the inertia of the whole robot for the current configuration.
             * According to the convention of this class about the layout of the
             * floating-base JSIM, this tensor is the 6x6 upper left corner of
             * the JSIM itself.
             * \return the 6x6 InertiaMatrix that correspond to the spatial inertia
             *   tensor of the whole robot, according to the last joints configuration
             *   used to update this JSIM
             */
            const InertiaMatrix& getWholeBodyInertia() const;
            /**
             * The matrix that maps accelerations in the actual joints of the robot
             * to the spatial force acting on the floating-base of the robot.
             * This matrix is the F sub-block of the JSIM in Featherstone's notation.
             * \return the 6x6 upper right block of this JSIM
             */
            const BlockF_t getF() const;
            /**
             * The submatrix of this JSIM related only to the actual joints of the
             * robot (as for a fixed-base robot).
             * This matrix is the H sub-block of the JSIM in Featherstone's notation.
             * \return the 6x6 lower right block of this JSIM,
             *   which correspond to the fixed-base JSIM
             */
            const BlockFixedBase_t getFixedBaseBlock() const;
        protected:
            /**
             * Computes and saves the inverse of the matrix L. See also computeL()
             */
            void computeLInverse();
        private:
            InertiaProperties& linkInertias;
            ForceTransforms* frcTransf;

            // The composite-inertia tensor for each link
            InertiaMatrix trunk_Ic;
            InertiaMatrix L_hip_Ic;
            InertiaMatrix L_thigh_Ic;
            const InertiaMatrix& L_shin_Ic;
            InertiaMatrix R_hip_Ic;
            InertiaMatrix R_thigh_Ic;
            const InertiaMatrix& R_shin_Ic;
            InertiaMatrix Ic_spare;

            MatrixType L;
            MatrixType Linv;
            MatrixType inverse;
        };


        inline const JSIM::MatrixType& JSIM::getL() const {
            return L;
        }

        inline const JSIM::MatrixType& JSIM::getInverse() const {
            return inverse;
        }

        inline const InertiaMatrix& JSIM::getWholeBodyInertia() const {
            return trunk_Ic;
        }

        inline const JSIM::BlockF_t JSIM::getF() const {
            return block<6, 6>(0, 6);
        }

        inline const JSIM::BlockFixedBase_t JSIM::getFixedBaseBlock() const {
            return block<6, 6>(6, 6);
        }


    }
}
#endif
