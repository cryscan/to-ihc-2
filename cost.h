//
// Created by cryscan on 8/18/21.
//

#ifndef TO_IHC_2_COST_H
#define TO_IHC_2_COST_H

#include "common.h"

#define INPUT_DIMS  state_dims + action_dims + action_dims
#define OUTPUT_DIMS 1

struct Cost : public ADBase<INPUT_DIMS, OUTPUT_DIMS> {
    using Base = ADBase<INPUT_DIMS, OUTPUT_DIMS>;
};

#undef INPUT_DIMS
#undef OUTPUT_DIMS

#endif //TO_IHC_2_COST_H
