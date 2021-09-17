#ifndef RCG_HOPPER_JSIM_H_
#define RCG_HOPPER_JSIM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>

#include "rbd_types.h"
#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"

namespace Hopper {
namespace rcg {

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot Hopper.
 */
    class JSIM : public iit::rbd::StateDependentMatrix<Hopper::rcg::JointState, 4, 4, JSIM> {
    private:
        typedef iit::rbd::StateDependentMatrix<Hopper::rcg::JointState, 4, 4, JSIM> Base;
    public:
        typedef Base::Scalar Scalar;
        typedef Base::Index Index;
        typedef Base::MatrixType MatrixType;
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

    protected:
        /**
         * Computes and saves the inverse of the matrix L. See also computeL()
         */
        void computeLInverse();
    private:
        InertiaProperties& linkInertias;
        ForceTransforms* frcTransf;

        // The composite-inertia tensor for each link
        InertiaMatrix u1_Ic;
        InertiaMatrix u2_Ic;
        InertiaMatrix body_Ic;
        const InertiaMatrix& leg_Ic;
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


}
}
#endif
